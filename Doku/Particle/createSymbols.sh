#!/bin/bash
#Add the path to directory which contains the Repository Blumentopf
repo_Path="$(pwd)/../.."

BlumentopfCPPLink="$repo_Path/Controller/Blumentopf.cpp"    # 
BlumentopfHPPLink="$repo_Path/Controller/Blumentopf.h"    # 


TimeCPPLink="$repo_Path/Controller/Time.h"    # 
echo "#################Create Symbol Links###################"
echo $BlumentopfCPPLink
##-------------Blumentopf.cpp .h-------------------------------
if [ -e "$BlumentopfCPPLink"  ]
then
  echo "$BlumentopfCPPLink already exists."
else
  echo "Create Symbol Link:$BlumentopfCPPLink."
  ln -s $repo_Path/libraries/Blumentopf/Blumentopf.cpp $repo_Path/Controller/Blumentopf.cpp
  echo
fi

if [ -e "$BlumentopfHPPLink"  ]
then
  echo "$BlumentopfHPPLink already exists."
else
  echo "Create Symbol Link:$BlumentopfCPPLink."
  ln -s $repo_Path/libraries/Blumentopf/Blumentopf.h $repo_Path/Controller/Blumentopf.h
  echo
fi

##-------------Time.cpp .h TimeLib.h-------------------------------

if [ -e "$repo_Path/Controller/Time.cpp"  ]
then
  echo "$repo_Path/Controller/Time.cpp already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/Time.cpp."
  ln -s $repo_Path/libraries/Time/Time.cpp $repo_Path/Controller/Time.cpp
  echo
fi

if [ -e "$repo_Path/Controller/Time.h"  ]
then
  echo "$repo_Path/Controller/Time.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/Time.h."
 ln -s $repo_Path/libraries/Time/Time.h $repo_Path/Controller/Time.h
  echo
fi

if [ -e "$repo_Path/Controller/TimeLib.h"  ]
then
  echo "$repo_Path/Controller/TimeLib.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/TimeLib.h."
 ln -s $repo_Path/libraries/Time/TimeLib.h $repo_Path/Controller/TimeLib.h
  echo
fi
##-------------LinkedList .h -------------------------------


if [ -e "$repo_Path/Controller/LinkedList.h"  ]
then
  echo "$repo_Path/Controller/LinkedList.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/LinkedList.h."
  ln -s $repo_Path/libraries/LinkedList/LinkedList.h $repo_Path/Controller/LinkedList.h
  echo
fi

##-------------RF24 .cpp .h nRF24L01.h R24_config.h printf.h-------------------------------


if [ -e "$repo_Path/Controller/RF24.h"  ]
then
  echo "$repo_Path/Controller/RF24.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/RF24.h."
  ln -s $repo_Path/libraries/RF24/RF24.h $repo_Path/Controller/RF24.h
  echo
fi

if [ -e "$repo_Path/Controller/RF24.cpp"  ]
then
  echo "$repo_Path/Controller/RF24.cpp already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/RF24.cpp."
  ln -s $repo_Path/libraries/RF24/RF24.cpp $repo_Path/Controller/RF24.cpp
  echo
fi

if [ -e "$repo_Path/Controller/nRF24L01.h"  ]
then
  echo "$repo_Path/Controller/nRF24L01.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/nRF24L01.h."
  ln -s $repo_Path/libraries/RF24/nRF24L01.h $repo_Path/Controller/nRF24L01.h
  echo
fi

if [ -e "$repo_Path/Controller/RF24_config.h"  ]
then
  echo "$repo_Path/Controller/RF24_config.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/RF24_config.h."
  ln -s $repo_Path/libraries/RF24/RF24_config.h $repo_Path/Controller/RF24_config.h
  echo
fi


if [ -e "$repo_Path/Controller/printf.h"  ]
then
  echo "$repo_Path/Controller/printf.h already exists."
else
  echo "Create Symbol Link:$repo_Path/Controller/printf.h."
  ln -s $repo_Path/libraries/RF24/printf.h $repo_Path/Controller/printf.h
  echo
fi
echo "---------------------Finished--------------------------"
#ln -s $(pwd)/../../libraries/Blumentopf/Blumentopf.cpp $(pwd)/../../Controller/Blumentopf.cpp


