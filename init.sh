mkdir data
cd data
mkdir outputs
wget https://dl.fbaipublicfiles.com/maskformer/mask2former/ade20k/panoptic/maskformer2_R50_bs16_160k/model_final_5c90d4.pkl
curl -o maskformer2_R50_bs16_160k.yaml https://raw.githubusercontent.com/facebookresearch/Mask2Former/main/configs/ade20k/panoptic-segmentation/maskformer2_R50_bs16_160k.yaml
curl -o Base-ADE20K-PanopticSegmentation.yaml https://raw.githubusercontent.com/facebookresearch/Mask2Former/main/configs/ade20k/panoptic-segmentation/Base-ADE20K-PanopticSegmentation.yaml
cd ../src/
git submodule init
git submodule update
cd Mask2Former
touch __init__.py