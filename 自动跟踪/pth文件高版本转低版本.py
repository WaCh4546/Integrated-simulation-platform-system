import torch
state_dict = torch.load("center_model.pth")
torch.save(state_dict, "center_model1.pth", _use_new_zipfile_serialization=False)