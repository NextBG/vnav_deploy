import math

from efficientnet_pytorch import EfficientNet
from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D

import torch
import torch.nn as nn

from .utils import replace_bn_with_gn

class Vnav(nn.Module):
    def __init__(
        self,
        enc_dim: int,
        context_size: int,
        pred_horizon: int,
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
            return self.vision_encoder(obs_imgs=kwargs["obs_imgs"], goal_vec=kwargs["goal_vec"], goal_mask=kwargs["goal_mask"])
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
        self.compress_obs_enc = nn.Linear(self.obs_encoder._fc.in_features, self.enc_dim)

        # Goal encoder
        self.goal_encoder = nn.Linear(2, self.enc_dim)
        
        # Self-attention
        self.positional_encoding = PositionalEncoding(self.enc_dim, max_len=self.context_size+1)
        self.sa_layer = nn.TransformerEncoderLayer(
            d_model=self.enc_dim,
            nhead=2,
            dim_feedforward=4*self.enc_dim,
            activation="gelu",
            batch_first=True,
        )
        self.sa_encoder= nn.TransformerEncoder(self.sa_layer, num_layers=4)

        # Goal mask
        goal_unmasked = torch.zeros((1, self.context_size+1), dtype=torch.bool)
        goal_masked = torch.zeros((1, self.context_size+1), dtype=torch.bool)
        goal_masked[0, -1] = 1.0
        self.goal_masks = torch.cat([goal_unmasked, goal_masked], dim=0)

        # Mean pool mask
        avg_unmasked = torch.ones((1, self.context_size+1), dtype=torch.float32)
        avg_masked = torch.ones((1, self.context_size+1), dtype=torch.float32)
        avg_masked[0, -1] = 0.0
        avg_masked = avg_masked * ((self.context_size+1)/(self.context_size))
        self.avg_masks = torch.cat([avg_unmasked, avg_masked], dim=0)

    def forward(self, 
                obs_imgs: torch.Tensor, 
                goal_vec: torch.Tensor,
                goal_mask: torch.Tensor,
            ) -> torch.Tensor:
        # Device
        device = obs_imgs.device

        # Batch size
        BS = obs_imgs.shape[0]
        
        # Encode goal vector
        goal_enc = self.goal_encoder(goal_vec)                                  # [B, 3] -> [B, E]
        goal_enc = goal_enc.unsqueeze(1)

        # Encode observations
        obs_imgs = obs_imgs.view(-1, 3, obs_imgs.shape[3], obs_imgs.shape[4])   # [B, N, C, H, W] -> [B*N, C, H, W]
        obs_enc = self.obs_encoder.extract_features(obs_imgs)                   # [B*N, F, _, _]
        obs_enc = self.obs_encoder._avg_pooling(obs_enc)                        # [B*N, F, 1, 1]
        obs_enc = obs_enc.view(BS*self.context_size, -1)                        # [B*N, F]
        obs_enc = self.compress_obs_enc(obs_enc)                                # [B*N, E]
        obs_enc = obs_enc.view((BS, self.context_size, self.enc_dim))           # [B, N, E]

        # Context encoding
        context_enc = torch.cat([obs_enc, goal_enc], dim=1)                     # [B, N+1, E]
        context_enc = self.positional_encoding(context_enc)                     # [B, N+1, E]

        mask = torch.index_select(self.goal_masks.to(device), 0, goal_mask)     # [B, N+1]
        context_token = self.sa_encoder(context_enc, src_key_padding_mask=mask) # [B, N+1, E]

        # Mean pool
        avg_mask = torch.index_select(self.avg_masks.to(device), 0, goal_mask)  # [B, N+1]
        context_token = context_token * avg_mask.unsqueeze(2)                   # [B, N+1, E]
        context_token = torch.mean(context_token, dim=1)                        # [B, E]

        return context_token
    
# Positional Encoding: https://pytorch.org/tutorials/beginner/transformer_tutorial.html
class PositionalEncoding(nn.Module): 
    def __init__(self, d_model, max_len=6):
        super().__init__()

        # Compute positional encoding
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