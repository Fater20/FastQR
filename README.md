# FastQR
## Introduction
We propose a new method to achieve fast pose estimation of objects based on multiple QR codes and monocular vision. We call it FastQR. Achieve 6DoF pose estimation of objects through post QR tag on them. You only need to know the relative positional relationship between the QR code and the object or ensure that any one QR code can be detected at the same time as another QR code.
We use the Apriltag as visual markers in this sample. You can also use other QR tags, for which you only need to replace the detection algorithm.

## Demo
Here is the demo video:
<video src="https://github.com/Fater20/FastQR/blob/main/video/Demo%20Video.mp4" controls="controls">
    <p>The video cannot be played normally. Please enter the video file to watch it.</p>
</video>

Here are the demo pictures:
</br>

|1|2|3|
|:----:|:----:|:----:|
|<img src="https://github.com/Fater20/FastQR/blob/main/image/Demo%20picture1.jpg" width="300" height="200" />|<img src="https://github.com/Fater20/FastQR/blob/main/image/Demo%20picture2.jpg" width="300" height="200" />|<img src="https://github.com/Fater20/FastQR/blob/main/image/Demo%20picture3.jpg" width="300" height="200" /> </br>|
|4|5|6|
|<img src="https://github.com/Fater20/FastQR/blob/main/image/Demo%20picture4.jpg" width="300" height="200" />|<img src="https://github.com/Fater20/FastQR/blob/main/image/Demo%20picture5.jpg" width="300" height="200" />|<img src="https://github.com/Fater20/FastQR/blob/main/image/Demo%20picture6.jpg" width="300" height="200" /> </br>|

## Note
1. This sample is built using the RZ/A2M development board of Renesas and the code is written in C. It's easy to implement this function on other development platforms.
2. We will update the version based on the Raspberry Pi platform in the future. We will also further improve the functions and provide examples of multi-joint objects such as robotic arms based on this method.
3. Any questions are welcome to be raised in Issues of this repository, and I will try my best to reply in time. Everyone is also welcome to participate in this project.