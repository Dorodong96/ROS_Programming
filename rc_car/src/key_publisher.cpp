// ---------------------------------------------------------------
// FILE    NAME : key_publisher.cpp
// DESCRIPTION  : 키보드 방향키(UP, DOWN, RIGHT) 모터 제어 신호 Publish 
//		      프로그램 종료 'q' 키
// NODE    NAME : key_publisher
// TOPIC   NAME : motor_msg
// MESSAGE TYPE : std_msgs/String
// ---------------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif


#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_RIGHT 0x43
#define KEYCODE_Q 0x71

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
       {
       ReadConsoleInput(handle, &buffer, 1, &events);
       if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
       else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
	else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopCar
{
public:
  TeleopCar();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  ros::Publisher  key_pub_;
};

TeleopCar::TeleopCar(){
  key_pub_ = nh_.advertise<std_msgs::String>("motor_msg", 1);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_publisher");

  TeleopCar teleop_car;

  signal(SIGINT,quit);

  teleop_car.keyLoop();

  quit(0);
  
  return(0);
}


void TeleopCar::keyLoop()
{
  char c;
  bool dirty=false;
  std_msgs::String motor_msg;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the RC-Car. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_UP:
        ROS_DEBUG("UP = FORWARD");
        motor_msg.data="forward";
        dirty = true;
        break;

      case KEYCODE_DOWN:
         ROS_DEBUG("DOWN = BACKWARD");
         //linear_ = -1.0;
	   motor_msg.data="backward";
         dirty = true;
         break;

      case KEYCODE_RIGHT:
         ROS_DEBUG("quit = STOP");
         motor_msg.data="stop";
         dirty = true;
         break;

      case KEYCODE_Q:
        ROS_DEBUG("quit = EXIT");
        return;
    }

    if(dirty ==true)
    {
      key_pub_.publish(motor_msg);    
      dirty=false;
    }
  }


  return;
}




