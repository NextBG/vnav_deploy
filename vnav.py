'''
Vector Navigator (VNAV) networt that infers next waypoint given past observations and goal vector
'''

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

import yaml
import numpy as np
from PIL import Image as PILImage
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

import torch

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
        self.obs_queue = []
        self.action_stats = torch.tensor([self.cfg["action_max_x"], self.cfg["action_max_abs_y"]]).to(self.device)
        self.busy = False

        # Model
        self.model = Vnav().to(self.device)
        # self.model.load_state_dict(torch.load("vnav/checkpoints/latest.pth"))
        self.model.eval()

        # Noise Scheduler
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=10,
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            prediction_type='epsilon'
        )

        # Publishers
        self.waypoint_publisher = rospy.Publisher("/waypoint", Vector3, queue_size=10)

        # Subscribers
        self.goal_subscriber = rospy.Subscriber("/goal_vec", Vector3, self._goal_callback)
        self.obs_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self._obs_callback)
        self.busy_flag_subscriber = rospy.Subscriber("/rover/controller_busy", Bool, self._busy_flag_callback)

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

# TODO: Debug this file

    def _busy_flag_callback(self, ctlr_busy: Bool):
        if ctlr_busy.data == False:
            if not self.busy:
                # Not enough obs
                if len(self.obs_queue) < self.cfg["context_size"]:
                    return
                
                # Busy flag
                self.busy = True
                
                print("Inference start")

                # Update obs queue
                self.obs_queue.pop(0)
                self.obs_queue.append(self.obs_curr)

                # Convert and Normalize obs
                obs_imgs_tensor = [torch.tensor(np.array(img).transpose(2, 0, 1), dtype=torch.float32) for img in self.obs_queue]
                obs_imgs = torch.stack(obs_imgs_tensor, dim=0).unsqueeze(0).to(self.device)  # [B=1, N, C=3, H, W]
            
                # Normalize goal
                goal_vec = torch.tensor([self.goal_vec.x, self.goal_vec.y]).unsqueeze(0).to(self.device) # [B=1, 2]
                goal_vec = self._normalize(goal_vec, self.action_stats)

                # Inference
                deltas = self._inference(obs_imgs, goal_vec)

                # Unnormalize
                deltas = self._unnormalize(deltas, self.action_stats)

                # Sum to get absolute waypoint
                trajs = torch.cumsum(deltas, dim=1)

                # Select trajectory
                wp_tensor = trajs[0] # TODO: Dummy

                # Select next waypoint
                next_waypoint = Vector3()
                next_waypoint.x = wp_tensor[0, 0].item()
                next_waypoint.y = wp_tensor[0, 1].item()

                # Publish
                self.waypoint_publisher.publish(next_waypoint)
                print(f"Waypoint published: {next_waypoint.x:.3f}, {next_waypoint.y:.3f}")

                # Free busy flag
                self.busy = False

            else:
                # print("Inference busy")
                pass

    def _normalize(self, trajs: torch.Tensor, action_stats: torch.Tensor) -> torch.Tensor:
        max_x, max_abs_y = action_stats

        # Normalize x from [0, ACTION_MAX_X] to [-1, 1]
        trajs[:, 0] = trajs[:, 0] / max_x * 2 - 1

        # Normalize y from [-ACTION_MAX_ABS_Y, ACTION_MAX_ABS_Y] to [-1, 1]
        trajs[:, 1] = trajs[:, 1] / max_abs_y

        return trajs
            
    def _unnormalize(self, trajs: torch.Tensor, action_stats: torch.Tensor) -> torch.Tensor: 
        max_x, max_abs_y = action_stats

        # Unnormalize x from [-1, 1] to [0, ACTION_MAX_X]
        trajs[:, :, 0] = (trajs[:, :, 0] + 1) / 2 * max_x

        # Unnormalize y from [-1, 1] to [-ACTION_MAX_ABS_Y, ACTION_MAX_ABS_Y]
        trajs[:, :, 1] = trajs[:, :, 1] * max_abs_y

        return trajs

    def _inference(self, obs_imgs: torch.Tensor, goal_vec: torch.Tensor) -> torch.Tensor:
        # Observation encoding
        context_token = self.model("vision_encoder", obs_imgs=obs_imgs, goal_vec=goal_vec)
        context_token = context_token.repeat(self.cfg["num_samples"], 1)

        # Noise sampling
        diffusion_out = torch.randn((self.cfg["num_samples"], self.cfg["pred_horizon"], 2), device=self.device)

        # Denoising
        for i in self.noise_scheduler.timesteps:
            noise_pred = self.model(
                "noise_predictor", 
                sample=diffusion_out, 
                timestep=i.repeat(self.cfg["num_samples"]).to(self.device), 
                global_cond=context_token
            )

            diffusion_out = self.noise_scheduler.step(
                model_output=noise_pred,
                timestep=i,
                sample=diffusion_out,
            ).prev_sample # [N, P, 2]

        return diffusion_out


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
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