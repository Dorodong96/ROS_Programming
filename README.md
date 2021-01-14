# ROS_Programming
*Basic and Tutorials of ROS Programming*

* 경북대학교 ROS 로보틱스 프로그래밍 수업을 기반으로 강의시간에 학습한 내용을 정리한 문서입니다.

* 교재는 **『ROS 로봇 프로그래밍』( 표윤석 외 / 루비페이퍼) **를 사용하였으며 본 문서에 주제별로 참고된 해당 페이지가 적혀있습니다.

* ROS와 관련된 모든 최신의 상세한 내용들은 https://wiki.ros.org/ 에 있습니다. 해당 페이지를 수시로 확인해보는 습관을 가집시다!

## [21.01.05]

### 1. ROS 설치

#### 1) ROS 설치

``` $ sudo apt-get install ros-kinetic-desktop-full```
(오류 발생 시 ```$ sudo rm -rf /var/lib/dpkg/lock-```)
```$ sudo apt-get install ros-kinetic-rqt*```
```$ sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential```

#### 2) ROS 의존성 패키지 설치 초기화 

```$ sudo rosdep init```
```$ rosdep update```
```$ ls –l /opt/ros/kinetic``` (확인)

#### 3) 설정 파일 만들기

```$ source /opt/ros/kinetic/setup.bash```   (*source 명령어 : source 스크립트 환경 파일)

#### 4) 작업폴더 생성 및 초기화

``` $ mkdir –p ~/catkin_ws/src```  (생성)
```$ cd ~/catkin_ws/src```
```$ catkin_init_workspace```  (초기화)
```$ cd ~/catkin_ws/```
```$ catkin_make```
```$ ls``` (build, devel, src 디렉토리 생성 확인)

#### 5) 작업 폴더 환경 설정 파일

``` $ source ~/catkin_ws/devel/setup.bash``` (~/.bashrc파일에 명령어 저장)
```$ vi ~/.bashrc``` (vim 편집기 활용, 터미널을 여는 순간 자동으로 실행)  ~ 설정 개념
해당 파일 가장 마지막에 다음 내용 작성 (단축키 개념) (32pg.)

```
# Set ROS Kinetic
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Set ROS Network
export ROS_HOSTNAME=localhost 
export ROS_MASTER_URI=http://$(ROS_HOSTNAME):11311

# Set ROS alias command
alias cw=’cd ~/catkin_ws’
alias cs=’cd ~/catkin_ws/src’
alias cm=’cd ~/catkin_ws && catkin_make’
```

#### 6) 변경한 설정 반영

```$ source ~/.bashrc```

#### 7) 적용된 환경설정 확인

```$ export | grep ROS```  (ROS 있는 것만 확인)

#### 8) 동작 테스트(38pg.)

```$ roscore```   (ROS 운영체제 구동)
새 터미널을 열고 (Ctrl + Alt + T), 

``` $ rosrun turtlesim turtlesim_node```
새 터미널을 열고, 
``` $ rosrun turtlesim turtle_teleop_key``` 

여기까지 되면 정상적으로 설치가 완료되었다고 볼 수 있다.

사용자 key는 수시로 업데이트 되므로 설치 시 사용자 key를 반드시 확인한다.

### 2. ROS Version

ROS의 버전에는 1.x 버전과 2.x 버전이 있다. 시장에서는 현재 기존의 1.x버전을 많이 사용 중이다. 프로젝트시에는 하드웨어에 따라 사용되는 버전이 다르므로 (예 : Jetson Nano : Ubuntu 18.x -> ROS1.x) 반드시 버전을 확인 후에 알맞은 버전의 ROS를 사용하도록 한다.

**<ROS2 특징>**

> * 다양한 플랫폼 -> 3대 OS
> * Real Time 지원 -> 환경 갖춰져야 함
> * 보안 강화 -> DDS(Data Distribution Service) –Security 지원
> * 통신 표준화 -> RTPS (Real Time Publish Subscribe)
> * 노드 매니저 
> * 다양한 언어 -> C++, Python 많이 사용 (요즘은 AI때문에 Python)
> * 클라이언트 라이브러리 (RCL : ROS Client Library)
> * API (Application Programming Interface): 함수들을 제공해 주겠다. 
> * 라이브러리(패키지) : 해당 OS, 언어에서 제공 or 외부에서 공개

### 3. IT 플랫폼의 변화
* 과거 : 제조사별 전용 하드웨어, 펌웨어, 서비스
* 최근 : 하드웨어 모듈, 운영체제, 앱, 유저 -> 분업 & 통합 ~ 빨리 만들고, 빨리 크자(Open) 
* 그러나 로봇은 아직 이런 시스템이 부족하다. (OS도 다양한데 아직 확실히 점유율 높은 것은 없음)

### 4. ROS의 특징

* 기본 운영체제 위에 설치되는 운영체제 ~ 엄밀히 따지면 소프트웨어 플랫폼
*  리눅스 커널을 사용하지만 BSD 라이선스 ~ 상용화에 이용해도 소스 공개할 의무 없음
*  메타운영체제 ~ 전통적인 운영체제 X, 기존의 OS기반으로 하는 SW프레임워크
  (디바이스 드라이버, 라이브러리, 디버그 도구, 메시지 통신, 구동 도구, 컴파일 도구, 인스톨러, 패키지 생성 및 릴리즈)
*  OS가 없는 보드(Arduino, 기타 임베디드)들은 Serial통신으로 제어신호 주고 받음 
* 공개 리포지토리 (각 패키지 별 저장소를 통한 공개) 
* API 형태 (ROS API 호출로 사용하던 코드에 삽입하도록 설계) 
* 복수의 프로그래밍 언어 지원 (클라이언트 라이브러리 통한 다양한 언어 지원) 
* 지원 사항 : 애플리케이션, 시뮬레이터, 지능모듈, 라이브러리, 디바이스드라이버, 디버그 툴 메시지 통신 등 
* https://wiki.ros.org/APIs : 프로그래밍 언어별로 API 자료 정리 
* https://robots.ros.org : 사용가능한 로봇 확인 
* https://wiki.ros.org/Sensors : 센서의 디바이스 드라이버 제공 
* 우분투 배포 기간에 맞추어 제공 (2년 주기)

### 5. 디렉토리
* ~/opt/ros/kinetic/ : 설치된 OS 디렉토리
* ~/catkin_ws/ : 사용할 프로젝트 디렉토리  

## [21.01.06]

### 1. 리눅스 파일 시스템

* ```$ pwd``` : print working directory / 현재 작업 디렉토리 
* ```$ ls ```: list show 
* ```$ ls –l``` : 자세히 보기 
* ```$ ls –a``` : 모두 보기 
* ```$ ls –al``` : 자세히 모두 보기  --> d : directory, l : link file, - : file

<디렉토리>
/dev : device의 약자, 입출력 장치 파일 관리
/etc : 시스템 환경 설정 파일 존재 (네트워크 관련 설정 파일, 사용자 정보 및 암호 정보, 파일 시스템 정보, 보안 파일, 시스템 초기화 파일 등 중요 설정 파일)

2. NODE 
• 실행 최소 단위의 프로세서 (클래스의 느낌이지만 하나의 프로세서로 정의됨)
• 하나의 실행 가능한 프로그램
• 하나의 목적 -> 하나의 노드
• 재사용 용이 (다른 곳에 사용 or 가져와서 사용 가능)
• 구성 : 이름, 토픽 이름, 서비스 이름, 메시지 형태, URI 주소 + 포트
• 노드들 간에 통신 가능

3. MASTER 
• NODE 요청에 따른 정보 등록 및 NODE 정보 제공 (NODE 정보를 다 갖고 있음)
• NODE & NODE 연결 (NODE들 사이에 정보를 전달)
• rescore 명령어로 구동
• HTTP기반 XMLRPC(XML Remote Procedure Call) 포로토콜 사용 
(XML : 태그활용, 간단한 명령어 주고받음, 데이터 전달 목적은 X)
• URI 주소 + 포트(11311) 할당 (/.bashrc 안에 Network 보면 정의 되어 있음)
• 동기화 데이터 : MASTER에서 파라미터 서버에 데이터 저장 (쓰레드 느낌)

4. PACKAGE
• ROS 구성 기본 단위 (기본 프로그램)
• ROS 응용프로그램 개발 단위 -> 최소 1개 NODE 이상 포함
• 설정 파일, 빌드 파일 존재 
• 메타 패키지 : 공통 목적을 가진 패키지 집합

5. MESSAGE
• NODE간 데이터를 주고 받는 형태
• 정수, 실수, 논리값, 배열 구조
• 통신 방법 : TCPROS, UDPROS ~ API형식으로 만든 Wrapper 클래스
(NODE-MASTER는 간단한 정보기 때문에 요청하면 알림)
• 분류 : 단방향 메시지(Topic), 양방향 메시지(Service, Action : 일을 어디까지 했다…)
(.msg, .srv, .action 파일, 교재 67pg.)

6. CATKIN
• Cmake 빌드 시스템을 ROS에 특화 시킨 ROS 빌드 시스템
• CMakeLists.txt.에 빌드 환경 기술
• ROS 관련 빌드, 패키지 관리, 패키지 간 의존관계 등

7. 특수 File
<설정 관련>
• CMakeLists.txt : ROS 빌드 시스템 Catkin에서 사용 / 빌드 환경 기술
• package.xml : 프로그램 단위인 Package의 정보를 담는 파일 / 패키지 이름, 저작자, 라이선스 ,의존성 패키지 정보

※	Message 관련 사용법 : http://wiki.ros.org/common_msgs 참고