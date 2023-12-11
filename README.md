# Off_boardpy

This is the package to run the SITL simulation. To implement all CIAFA architecture see [Felix](https://github.com/mg-felix/sitl-simulations)

# Install before start

GStreamer


```bash
apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

Geographiclib

```bash
sudo geographiclib-get-geoids egm96-5
```

Push to scripts this git [TrainYourOwnYolo](https://github.com/AntonMu/TrainYourOwnYOLO)

Inside TrainYourOwnYOLO/Data/Model_Weights/ do the following command

```bash
gdown https://drive.google.com/uc?id=1MGXAP_XD_w4OExPP10UHsejWrMww8Tu7
```

Introduze in .bashrc

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/mnt/hdd_1_500gb/dasilva/fod_work/venv/lib/python3.8/site-packages/nvidia/cuda_runtime/lib/:/mnt/hdd_1_500gb/dasilva/fod_work/venv/lib/python3.8/site-packages/nvidia/cublas/lib:/mnt/hdd_1_500gb/anaconda3/pkgs/cudatoolkit-10.1.243-h6bb024c_0/lib/:/mnt/hdd_1_500gb/dasilva/fod_work/venv/lib/python3.8/site-packages/nvidia/cudnn/lib/:/usr/local/cuda-11.2/targets/x86_64-linux/lib/
```
