pip install pyrealsense2
python create_meshes.py
ls
cd FoundationPose
ls
python create_meshes.py
python run_lego.py
pip install transformers accelerate
python run_lego.py
find weights -name "*.pth"
python run_lego.py
cat estimater.py | grep "def __init__"
python run_lego.py
bash build_all.sh
python run_lego.py
python create_block.py
python run_lego.py
python check_depth.py
python run_lego.py
python run_pca.py
# 1. 进入代码挂载的目录（根据 FoundationPose 的常见镜像设置，通常在 /workspace）
cd /workspace/FoundationPose
# 2. 运行乐高积木识别与抓取代码
python run_lego.py
ls
cd FoundationPose
python run_lego.py
xhost +local:docker
# 1. 安装 x11-xserver-utils 以获得 xhost 命令
sudo apt-get update
sudo apt-get install x11-xserver-utils -y
# 2. 授予 Docker 访问显示器的权限
xhost +local:docker
# 1. 安装 x11-xserver-utils 以获得 xhost 命令
sudo apt-get update
sudo apt-get install x11-xserver-utils -y
# 2. 授予 Docker 访问显示器的权限
xhost +local:docker
sudo apt-get update
# 退出当前报错的容器
exit
