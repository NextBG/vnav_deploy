import rospy
from sensor_msgs.msg import Image

from PIL import Image as PILImage
import numpy as np

import torch

class ImageDebug:
    def __init__(self):
        rospy.init_node('image_debug')

        # Params
        self.img = None

        # Subscribers
        self.obs_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self._obs_callback)

    def _obs_callback(self, img: Image):
        self.img = img

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.img is not None:
                # Convert to PIL image (RGB8)
                height, width = self.img.height, self.img.width # [360, 640]

                pil_img = PILImage.frombytes("RGB", (width, height), self.img.data)

                # Resize
                pil_img = pil_img.resize((256, 144))

                # Save
                # pil_img.save("debug.png")

                # Convert to tensor
                img_np = np.array(pil_img).transpose(2, 0, 1)
                img_tensor = torch.tensor(img_np, dtype=torch.float32).unsqueeze(0)

                print(img_tensor.shape)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        image_debug = ImageDebug()
        image_debug.run()
    except rospy.ROSInterruptException:
        pass