# RobotPuppetry

## About The Project

[![Project Screen Shot][project-screenshot]]()

This repository contains a series of code for Robot puppetry using the  [OpenCV library](https://opencv.org/) and the [Dlib library](http://dlib.net/) to mimick head and face movements onto a [Poppy Ergo Jr robot](https://www.poppy-project.org/en/robots/poppy-ergo-jr/).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![C++][cpp-shield]][cpp-url]
* [![OpenCV][opencv-shield]][opencv-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

To set up the project locally, you need to install (if not already the case) some dependencies.
To get a local copy up and running follow these steps.

### Prerequisites

* C++ Compiler

Install the build-essential package
  ```sh
  sudo apt install build-essential 
  ```
  
* OpenCV

Install the dev library
  ```sh
  sudo apt install libopencv-dev 
  ```
  
  * Dynamixel SDK
  
 Download the Dynamixel SDK
 ```sh
 git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
 ```
 
 Go to the c++/build folder
  ```sh
 cd DynamixelSDK/c++/build/linux64
 ```

 Compile and install
  ```sh
 make
 sudo make install
 ```
 
 Upgrade USB access privileges by aading your account to the dialout group
   ```sh
  sudo usermod -aG dialout <your_account_id>
 ```
 
   * Dlib
  
 Download the Dlib library
 ```sh
 git clone https://github.com/davisking/dlib.git
 ```
 or
here [dlib](http://dlib.net/files/dlib-19.24.zip)

   * X11
  
 Download the X11 dev library
 ```sh
 sudo apt install libX11-dev
 ```
 
### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/GuillaumeGibert/RobotPuppetry.git
   ```
2. Open a terminal
3. Compile/Link by calling the makefile
 ```sh
   make
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

1. Open a terminal
2. Launch the puppetry executable
```sh
./bin/robotpuppetry 
```


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LICENSE -->
## License

Distributed under the GPL License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Guillaume Gibert

Project Link: [https://github.com/GuillaumeGibert/RobotPuppetry](https://github.com/GuillaumeGibert/RobotPuppetry)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[arduino-shield]: https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white
[arduino-url]: https://www.arduino.cc/
[python-shield]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[python-url]: https://www.python.org/
[opencv-shield]: https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white
[opencv-url]: https://opencv.org/
[cpp-shield]: https://img.shields.io/badge/-C++-blue?logo=cplusplus
[cpp-url]: https://isocpp.org/

[project-screenshot]: images/screenshot.png

[contributors-shield]: https://img.shields.io/github/contributors/GuillaumeGibert/RobotPuppetry.svg?style=for-the-badge
[contributors-url]: https://github.com/GuillaumeGibert/RobotPuppetry/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/GuillaumeGibert/RobotPuppetry.svg?style=for-the-badge
[forks-url]: https://github.com/GuillaumeGibert/RobotPuppetry/network/members
[stars-shield]: https://img.shields.io/github/stars/GuillaumeGibert/RobotPuppetry.svg?style=for-the-badge
[stars-url]: https://github.com/GuillaumeGibert/RobotPuppetry/stargazers
[issues-shield]: https://img.shields.io/github/issues/GuillaumeGibert/RobotPuppetry.svg?style=for-the-badge
[issues-url]: https://github.com/GuillaumeGibert/RobotPuppetry/issues
[license-shield]: https://img.shields.io/github/license/GuillaumeGibert/RobotPuppetry.svg?style=for-the-badge
[license-url]: https://github.com/GuillaumeGibert/RobotPuppetry/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/guillaume-gibert-06502ba4