import math

from efficientnet_pytorch import EfficientNet
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D

import torch
import torch.nn as nn

from .utils import replace_bn_with_gn

class Vnav(nn.Module):
    def __init__(
        self,
        enc_dim: int = 256,
        context_size: int = 2,
        pred_horizon: int = 8,
        ):
        # Init
        super().__init__()

        # Vision encoder
        self.vision_encoder = VisionEncoder(
            enc_dim=enc_dim,
            context_size=context_size
        )

        # Noise prediction network
        self.noise_predictor = ConditionalUnet1D(
            input_dim=2,
            global_cond_dim=enc_dim,
            down_dims=[64, 128, 256],
            n_groups=pred_horizon,
        )
    
    def forward(self, func_name: str, **kwargs):
        if func_name == "vision_encoder" :
            return self.vision_encoder(obs_imgs=kwargs["obs_imgs"], goal_vec=kwargs["goal_vec"])
        if func_name == "noise_predictor":
            return self.noise_predictor(sample=kwargs["sample"], timestep=kwargs["timestep"], global_cond=kwargs["global_cond"])        

class VisionEncoder(nn.Module):
    def __init__(
            self,
            enc_dim: int,
            context_size: int,
    ):
        # Init
        super().__init__()
        self.enc_dim = enc_dim
        self.context_size = context_size

        # Observation encoder
        self.obs_encoder = EfficientNet.from_name("efficientnet-b0", in_channels=3)
        replace_bn_with_gn(self.obs_encoder)
        
        # Compression
        self.compress_obs_enc = nn.Linear(1920, self.enc_dim)

        # Goal encoder
        self.goal_encoder = nn.Linear(2, self.enc_dim)
        
        # Initialize positional encoding and self-attention layers
        self.positional_encoding = PositionalEncoding(self.enc_dim, max_len=self.context_size + 1)
        self.sa_layer = nn.TransformerEncoderLayer(
            d_model=self.enc_dim,
            nhead=2,
            dim_feedforward=4*self.enc_dim,
            activation="gelu",
            batch_first=True,
        )
        self.sa_encoder= nn.TransformerEncoder(self.sa_layer, num_layers=2)

    def forward(self, 
                obs_imgs: torch.tensor, 
                goal_vec: torch.tensor
            ) -> torch.tensor:
        # Batch size
        BS = obs_imgs.shape[0]
        
        # Encode goal vector
        goal_enc = self.goal_encoder(goal_vec) # [B, 3] -> [B, enc]
        goal_enc = goal_enc.unsqueeze(1)

        # Encode observations
        obs_imgs = obs_imgs.view(-1, 3, obs_imgs.shape[3], obs_imgs.shape[4])   # [B, N, C, H, W] -> [B*N, C, H, W]
        obs_enc = self.obs_encoder.extract_features(obs_imgs)                   # [B*N, F, _, _]
        obs_enc = self.obs_encoder._avg_pooling(obs_enc)                        # [B*N, F, 1, 1]
        obs_enc = obs_enc.view(BS*self.context_size, -1)                        # [B*N, F]
        obs_enc = self.compress_obs_enc(obs_enc)                                # [B*N, E]
        obs_enc = obs_enc.view((BS, self.context_size, self.enc_dim))           # [B, N, E]

        # Context encoding
        context_enc = torch.cat([obs_enc, goal_enc], dim=1)
        context_enc = self.positional_encoding(context_enc) # [B, context_size+2, obs_enc_size]

        # Self-attention
        context_token = self.sa_encoder(context_enc) # [B, C+1, E]

        # Average pooling
        context_token = torch.mean(context_token, dim=1)

        return context_token
    
# Positional Encoding: https://pytorch.org/tutorials/beginner/transformer_tutorial.html
class PositionalEncoding(nn.Module): 
    def __init__(self, d_model, max_len=6):
        super().__init__()

        # Compute the positional encodings once in log space.
        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        pe = torch.zeros(1, max_len, d_model)
        pe[0, :, 0::2] = torch.sin(position * div_term)
        pe[0, :, 1::2] = torch.cos(position * div_term)
        self.register_buffer('pe', pe)

    def forward(self, x):
        # Add to input
        x = x + self.pe[:, :x.size(1), :]
        return x
    
# Test code
if __name__ == "__main__":
    # Hyperparameters
    BS = 1
    SAMPLE_NUM = 10
    PRED_HORIZON = 8
    CONTEXT = 2

    # Device
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Inputs
    obs_imgs = torch.randn((BS, CONTEXT, 3, 144, 256), device=device) # [B, N, C, H, W]
    goal_vec = torch.randn((BS, 2), device=device) # [B, 2]

    # Model
    model = Vnav(
        enc_dim=256,
        context_size=CONTEXT,
        pred_horizon=PRED_HORIZON
    ).to(device)

    # Noise Scheduler
    noise_scheduler = DDPMScheduler(
        num_train_timesteps=10, # Diffusion iterations
        beta_schedule='squaredcos_cap_v2',
        clip_sample=True,
        prediction_type='epsilon'
    )

    # Inference
    model.eval()
    with torch.no_grad():
        # Observation encoding
        context_token = model("vision_encoder", obs_imgs=obs_imgs, goal_vec=goal_vec)

        # Dulpicate context token to sample num
        context_token = context_token.repeat(SAMPLE_NUM, 1)
        diffusion_out = torch.randn((SAMPLE_NUM, PRED_HORIZON, 2), device=device)

        # Action sampling
        for i in noise_scheduler.timesteps:
            print(i)
            noise_pred = model(
                "noise_predictor", 
                sample=diffusion_out, 
                timestep=i.repeat(SAMPLE_NUM).to(device), 
                global_cond=context_token
            )

            diffusion_out = noise_scheduler.step(
                model_output=noise_pred,
                timestep=i,
                sample=diffusion_out,
            ).prev_sample

        print(diffusion_out.shape)