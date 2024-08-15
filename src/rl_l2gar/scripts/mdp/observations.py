import sys
import rospy
import rospkg
import torch
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('rl_l2gar')+ '/scripts')
import utils.math_utils as math_utils


def projected_gravity_b(base_quat_w : torch.Tensor, gravity_vec: torch.Tensor = torch.tensor([0, 0, -1])) -> torch.Tensor:
    '''
      Quaternion Format: [w, x, y, z]
      Gravity Vector: [x, y, z], Z是负的. 在lab中, 对gravity_vec做了normalize处理, 最终的gravity_vec是[0, 0, -1]
      传入的base_quat_w和gravity_vec可以是1维的, 也可以是2维的, 如果是2维的, 第一维的大小必须是1. 最终返回的是一个1维tensor
    '''
    if gravity_vec.norm(p=2, dim = -1)[-1].item() >=1+1e-3 or gravity_vec.norm(p=2, dim = -1)[-1].item() <=1-1e-3:
      gravity_vec = gravity_vec / gravity_vec.norm(p=2,dim=-1).clamp(min = 1e-9, max=None)
    if base_quat_w.dim() == 1:
      base_quat_w = base_quat_w.unsqueeze(0)
    if gravity_vec.dim() == 1:
      gravity_vec = gravity_vec.unsqueeze(0)
    if base_quat_w.shape[0] != 1 or gravity_vec.shape[0] != 1:
      raise ValueError("The first dimension of base_quat_w and gravity_vec must be 1")
    return math_utils.quat_rotate_inverse(base_quat_w, gravity_vec).squeeze()
    