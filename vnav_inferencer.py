'''
Vector Navigator (VNAV) networt that infers next waypoint given past observations and goal vector
'''

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, Joy

import os
import time
import yaml
import numpy as np
from collections import deque
from PIL import Image as PILImage
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import torch
from torchvision import transforms

from vnav.model import Vnav

class VnavInferencer:
    def __init__(self, cfg: dict):
        rospy.init_node('vnav_inferencer')

        # Device
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        # Params
        self.cfg = cfg
        self.goal_vec = Vector3()
        self.obs_curr = Image()
        self.joy_msg = Joy()
        self.obs_queue = []
        self.action_stats = torch.tensor([self.cfg["action_max_x"], self.cfg["action_max_abs_y"]]).to(self.device)
        self.busy = False
        self.ready_to_pub = False
        self.ctlr_busy = Bool()
        self.action_queue = deque(maxlen=self.cfg["action_queue_size"])
        self.action_queue_size = self.cfg["action_queue_size"]

        # Model
        self.model = Vnav(
            enc_dim=self.cfg["encoding_dim"],
            context_size=self.cfg["context_size"],
            pred_horizon=self.cfg["pred_horizon"],
        ).to(self.device)
        self.model.load_state_dict(torch.load("vnav/checkpoints/latest.pth", map_location=self.device))
        self.model.eval()
        print("Model loaded")

        # Noise Scheduler
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=10,
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            prediction_type='epsilon'
        )

        # Transform
        self.img_tf = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                std=[0.229, 0.224, 0.225]),
        ])

        # Publishers
        self.waypoint_publisher = rospy.Publisher("/waypoint", Vector3, queue_size=10)

        # Subscribers
        self.goal_subscriber = rospy.Subscriber("/goal_vec", Vector3, self._goal_callback)
        self.obs_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self._obs_callback)
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self._joy_callback)
        self.busy_flag_subscriber = rospy.Subscriber("/rover/controller_busy", Bool, self._busy_flag_callback)

        # Log folder
        self.log_idx = 0
        self.log_dir = os.path.join("logs", time.strftime("%y%m%d-%H%M%S"))
        os.makedirs(self.log_dir, exist_ok=True)

        print("VNAV inferencer node initialized")

    def _goal_callback(self, goal_vec: Vector3):
        # Update
        self.goal_vec = goal_vec

    def _obs_callback(self, img: Image):
        # Convert to np array
        img_pil = PILImage.frombytes("RGB", (img.width, img.height), img.data)
        img_pil = img_pil.resize((256, 144))

        # Update
        self.obs_curr = img_pil

        # Initialize queue
        if len(self.obs_queue) == 0:
            for _ in range(self.cfg["context_size"]):
                self.obs_queue.append(img_pil)

    def _joy_callback(self, joy_msg: Joy):
        self.joy_msg = joy_msg

    def _visualize(self, obs_queue: list, goal_vec: torch.Tensor, traj: torch.Tensor):
        # Detach
        goal_vec = goal_vec.detach().cpu().numpy()
        traj = traj.detach().cpu().numpy()

        # Figure
        fig, axs = plt.subplots(1, 2, figsize=(10, 5))

        # Title
        fig.suptitle(f"Goal: {goal_vec[0]:.3f}, {goal_vec[1]:.3f}")

        # Latest observation
        axs[0].set_title("Last observation")
        axs[0].imshow(obs_queue[-1])

        # Goal
        axs[1].set_title("Trajectories")
        axs[1].grid()
        axs[1].invert_xaxis()
        axs[1].set_aspect('equal', adjustable='box')
        axs[1].plot(goal_vec[1], goal_vec[0], "bx")

        # Trajectories
        traj = np.concatenate([np.zeros((traj.shape[0], 1, 2)), traj], axis=1)
        for i in range(traj.shape[0]):
            axs[1].plot(traj[i, :, 1], traj[i, :, 0], "r-o", alpha=0.5, markersize=3)

        # Log
        plt.savefig(os.path.join(self.log_dir, f"{self.log_idx:06d}.png"))
        plt.savefig("visualize.png")
        plt.close()
        self.log_idx += 1

    def _busy_flag_callback(self, ctlr_busy: Bool):
        # Controller status
        self.ctlr_busy = ctlr_busy

        # Both controller and inferencer are not busy
        if ctlr_busy.data == False and not self.busy:
            # Queue not empty
            if len(self.action_queue) > 0:
                return

            # Not enough obs
            if len(self.obs_queue) < self.cfg["context_size"]:
                return
            
            # Zero waypoint
            if self.goal_vec.x == 0 and self.goal_vec.y == 0:
                return

            # Busy flag
            self.busy = True
            print(f"Inference start, goal: {self.goal_vec.x:.3f}, {self.goal_vec.y:.3f}")

            # Update obs queue
            self.obs_queue.pop(0)
            self.obs_queue.append(self.obs_curr)

            # Normalize obs
            n_obs_imgs = [self.img_tf(img) for img in self.obs_queue]
            n_obs_imgs = torch.stack(n_obs_imgs, dim=0).to(self.device).unsqueeze(0)  # [B=1, N, C=3, H, W]
        
            # Goal vector
            goal_vec = torch.tensor([self.goal_vec.x, self.goal_vec.y], device=self.device).unsqueeze(0) # [B=1, 2]

            # Normalize
            dist = torch.norm(goal_vec, dim=1, keepdim=True)                    # [B=1, 1]  
            n_dist = torch.tanh(dist * self.cfg["goal_norm_factor"])            # [B=1, 1]
            n_coeff = n_dist / dist                                             # [B=1, 1]
            n_goal_vec = goal_vec * n_coeff                                     # [B=1, 2]

            # Goal mask
            goal_mask = torch.zeros((1,), device=self.device).int() # [B=1, 1]

            # Inference
            n_deltas = self._inference(n_obs_imgs, n_goal_vec, goal_mask)

            # Unnormalize
            deltas = self._unnormalize(n_deltas, self.action_stats)

            # Sum to get absolute waypoint
            trajs = torch.cumsum(deltas, dim=1)

            # Visualize
            self._visualize(self.obs_queue, goal_vec[0], trajs)

            # Select trajectory
            wp_tensor = deltas[0] # TODO: Dummy

            # Append to action queue
            for i in range(self.action_queue_size):
                self.action_queue.append(wp_tensor[0].detach().cpu().numpy())
            
            self.ready_to_pub = True

    def _normalize(self, data: torch.Tensor, stats: torch.Tensor) -> torch.Tensor:
        x_max, y_abs_max = stats
        data[:, 0] = data[:, 0] / x_max * 2 - 1     # x [0, ACTION_MAX_X] -> [-1, 1]
        data[:, 1] = data[:, 1] / y_abs_max         # y [-ACTION_MAX_Y, ACTION_MAX_Y] -> [-1, 1]
        return data
            
    def _unnormalize(self, ndata: torch.Tensor, stats: torch.Tensor) -> torch.Tensor: 
        x_max, y_abs_max = stats
        ndata[:, :, 0] = (ndata[:, :, 0] + 1) / 2 * x_max  # x [-1, 1] -> [0, ACTION_MAX_X]
        ndata[:, :, 1] = ndata[:, :, 1] * y_abs_max        # y [-1, 1] -> [-ACTION_MAX_Y, ACTION_MAX_Y]
        return ndata

    def _inference(self, n_obs_imgs: torch.Tensor, n_goal_vec: torch.Tensor, goal_mask: torch.Tensor) -> torch.Tensor:
        # Observation encoding
        context_token = self.model(
            "vision_encoder", 
            obs_imgs=n_obs_imgs, 
            goal_vec=n_goal_vec, 
            goal_mask=goal_mask
        )
        context_token = context_token.repeat(self.cfg["num_samples"], 1)

        # Noise sampling
        diffusion_out = torch.randn((self.cfg["num_samples"], self.cfg["pred_horizon"], 2), device=self.device)

        # Denoise
        for i in self.noise_scheduler.timesteps:
            # Predict noise
            noise_pred = self.model(
                "noise_predictor", 
                sample=diffusion_out, 
                timestep=i.repeat(self.cfg["num_samples"]).to(self.device), 
                global_cond=context_token
            )

            # Remove noise
            diffusion_out = self.noise_scheduler.step(
                model_output=noise_pred,
                timestep=i,
                sample=diffusion_out,
            ).prev_sample # [N, H, 2]

        return diffusion_out


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.busy and self.ready_to_pub:
                # Controller busy
                if self.ctlr_busy.data:
                    continue

                if len(self.action_queue) > 0:
                    print(f"Action queue: {len(self.action_queue)}/{self.action_queue_size}")

                    # Pop
                    wp = self.action_queue.popleft()

                    # Action
                    wp_to_pub = Vector3()
                    wp_to_pub.x = wp[0]
                    wp_to_pub.y = wp[1]

                    # Wait for confirmation
                    print(f"Action to pub: {wp_to_pub.x:.3f}, {wp_to_pub.y:.3f}, RB to confirm...", end="")
                    while(self.joy_msg.buttons[5] != 1):
                        continue

                    # Publish
                    self.waypoint_publisher.publish(wp_to_pub)
                    self.ctlr_busy.data = True
                    print("Published")

                else:
                    # Reset
                    self.ready_to_pub = False
                    self.busy = False

            rate.sleep()

if __name__ == "__main__":
    # Load config
    with open("vnav/vnav_config.yaml", "r") as f:
        config = yaml.safe_load(f)

    # ROS
    try:
        vnav_inferencer = VnavInferencer(config)
        vnav_inferencer.run()
    except rospy.ROSInterruptException:
        pass
