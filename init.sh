mkdir data
cd data
wget https://dl.fbaipublicfiles.com/maskformer/mask2former/ade20k/panoptic/maskformer2_R50_bs16_160k/model_final_5c90d4.pkl
curl -o maskformer2_R50_bs16_160k.yaml https://raw.githubusercontent.com/facebookresearch/Mask2Former/main/configs/ade20k/panoptic-segmentation/maskformer2_R50_bs16_160k.yaml
