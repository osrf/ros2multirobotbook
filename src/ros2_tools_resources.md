ROS Resources
=============

The most up to date information about ROS can be found on the web. There are a
myriad of resources on-line to help you out in your educational or
practical journey. One thing to keep in mind is that ROS, like most software,
has different versions, and the format and structure of commands and API calls
may differ slightly between versions (although the developers try to keep things
as stable as possible). This book is specifically written for _ROS 2, Eloquent
Elusor_, or ROS Eloquent to be terse. While newer or older versions of ROS will
be generally helpful it is worth paying attention to the version number as there
can be minor changes between versions. ROS has both major versions (i.e. ROS
1, and ROS 2) and minor versions denoted by a pair of matching letter adjectives
and specific nouns related to specific genus and species of turtles
(e.g. Eloquent Elusor, or Foxy Fitzroy). The biggest difference in the CLI and
API come between the major versions, i.e. ROS 1 and ROS 2. There may be minor
changes between minor versions, and usually it is the addition of features, not
their modification or removal. It is worth noting that ROS versions are usually
pegged to specific version of Ubuntu Linux. If your search engine results are
specific enough for your particular problem it is a good practice to append your
ROS version to your search. Moreover, when seeking help or posting questions
online you should always specify the version of ROS version you are using.


ROS grew up with the modern web, and as such it has a variety of resources and
forums to help you solve problems and learn about the API and tools. Some of our
web resources actually pre-date more widely used systems, so it helps to know
where they are and how to use them. Probably the most important resource on the
web for ROS users is [answers.ros.org](http://anwers.ros.org). Answers is a Q&A
website similar to StackOverflow. Once you register for Answers you can ask or
answer any ROS-related question. Be aware that asking a question well can be
difficult. You should include as much information as possible to help others
answer your question. This means you should include the ROS version, any
debugging or stack trace information you have, and the offending source code.

Aside from ROS Answers you should check out both the ROS 2 tutorials and API
documentation, and the ROS 1 wiki. The ROS 1 wiki can be found at
[wiki.ros.org](http://wiki.ros.org/). While it is specifically
dedicated to ROS 1, much of the information is still relevant to ROS 2. If you
are searching for up to date ROS 2 information, your go to source is the ROS 2 tutorials and API documents located at
[index.ros.org/doc/ros2](https://index.ros.org/doc/ros2/). Many of
the tutorials you will find in this book pull directly from this body of
work. If you would like to find the latest ROS news and discuss various ROS
features, the ROS Discourse forum at
[discourse.ros.org](https://discourse.ros.org/) is your best bet. ROS
discourse is the community hub where developers discuss their latest projects
and debate the finer points of ROS development.

For ROS application developers there are a number of tools to help you connect
with the broader ROS developer community. Open Robotics supports
[index.ros.org](https://index.ros.org/), which is an extended list of ROS
packages sorted by version. If you are searching for a ROS driver for a
particular piece of hardware, then the index is a great place to start. If you
find a package with failing tests, or would like to know the build status of any
ROS package, take a look at [build.ros.org](http://build.ros.org/). Similarly, for un-indexed
packages [GitHub maintains a ROS code tag](https://github.com/topics/ros?o=desc&s=updated).
This tag will allow you to search all of the tagged repositories that are publicly listed.
At the time of writing there were close to 4000 repositories listed on GitHub, so there is a
pretty good chance you'll find what you need.


Finally, there are a variety of unofficial resources that you should be aware of
that can be useful, particularly if you want to keep yourself up to date with
the latest ROS projects and features. Both [Open Robotics](https://twitter.com/openroboticsorg) and
[ROS](https://twitter.com/rosorg) maintain  twitter feeds to share the latest
news. We also have a yearly ROS developers conference called
[ROSCon](https://roscon.ros.org/2020/); most talks are freely available
on the web. There are a few other resources that can also be useful including the [ROS subreddit](https://www.reddit.com/r/ROS/)
and an "unofficial" [ROS Discord](https://discord.com/invite/HnVcz5a).

Setting Up Your Computer
===

For this chapter we assume that you are working on a modern desktop with a
discrete graphics card. While a graphics card isn't necessary for this chapter,
later chapters will be graphics intensive and having one will greatly improve
the end user experience. Moreover, this book assumes you are working with the
Ubuntu Linux 20.04 operating system. While other operating systems are supported
by ROS 2, all of the tutorials and instructions on this book assume you are
running Linux. If instead you're using a Mac or Windows PC, you can install ROS 2
Eloquent Elusor using the instructions found on the [ROS 2 installation instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/)
page. An alternative path for installation on Mac and PC is to using a virtual
machine. Roughly the process for doing so is as follows:

1. Install virtual machine software like [Virtual
   Box](https://www.virtualbox.org/) or
   [VMWare](https://www.vmware.com/products/workstation-pro.html) on your host
   machine.
1. Create a virtual machine using the software, and install [Desktop Ubuntu 18.04 Bionic
   Beaver from the Canonical website.](https://ubuntu.com/download/desktop)
   Configure the installation as you wish.
1. Now start your virtual machine and log in as a user. The directions below
   should be applicable.

For these initial tutorials we will be working with the pre-compiled ROS 2:
Eloquent Elusor desktop version. These directions follow directly from the
installation instructions found on the [Eloquent release
page]https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/). To
run these commands you'll need a terminal window. To open a terminal in Ubuntu
18.04 click on the nine dots in the bottom left hand of the screen. A dialog
should appear. Enter the word _terminal_ and click on the terminal icon to open
a terminal. Alternatively, you can press the control, alt, and 't' keys
simultaneously to open a terminal (we abbreviate this `CTRL-ALT-T`).


Setup Locale
============

The first step is to make sure you have a locale which supports `UTF-8`. What this means is that we
will check that the language used by your computer uses a particular format of
text. If you are in a minimal environment, such as a docker container, the locale may be
something minimal like POSIX. We test with the following settings. It
should be fine if you're using a different UTF-8 supported locale.

``` {.sourceCode .bash}
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Setup Sources
=============

You will need to add the ROS 2 apt repositories to your system. Out of the box
Ubuntu doesn't know where the ROS 2 binary programs live so we have to give it a
secure location. To do this the computer will prompt you for your root
password. For more technical readers we need to authorize the ROS GPG key with
apt by typing the following command in the terminal:

``` {.sourceCode .bash}
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```


Install ROS 2 packages
======================

The next steps for installing ROS is to do a system update (i.e. check for newer
programs) and then install ROS Eloquent. To do this we run the following
commands. Be aware that these commands will download a lot of data and may take
awhile. It is best to run these commands on your home network.

``` {.sourceCode .bash}
sudo apt update
```

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

``` {.sourceCode .bash}
sudo apt install ros-eloquent-desktop
```

Next we'll install a set of tutorials called `TurtleSim`. To do this we run
another apt command.

``` {.sourceCode .bash}
sudo apt install ros-eloquent-turtlesim
```

ROS 2 command line tools use argcomplete for autocompletion. If you
want autocompletion, installing argcomplete is necessary. We're also going to
install a few other tools to make our lives easier.

``` {.sourceCode .bash}
sudo apt install python3-argcomplete htop byobu
```

Check Your Installation
=================

ROS uses `environment variables` to help keep track of what version of ROS is
running and where all the programs using ROS are used on the computer. To set
these environment variable we `source`, or load, a bash script file. A bash
script file isn't magic; it is just a series of commands to enter into the
terminal, just like the series of commands we just entered to setup ROS. It is
possible to have different version of ROS running on a single computer. Using
the wrong version of ROS can lead to all sorts of problems and is a common
mistake for new users! If you are having problems try sourcing the correct ROS
bash file. From now on, whenever you open a new terminal, you will
need to tell the computer which version of ROS to use. To set the necessary
environment variables for ROS you need to `source` a bash file every time you
open a new terminal. Yes, this is annoying, but it is a sound approach as it
makes the version of ROS you are using explicit. On Ubuntu 18.04 all versions of
ROS live in `/opt/ros/`. Inside this directory will be a programs and script
files to run ROS. To tell the operating system that we want to use ROS Eloquent
we simply source the ROS Eloquent setup.bash file using the command below.


``` {.sourceCode .bash}
source /opt/ros/eloquent/setup.bash
```

Once that command runs your terminal should be ready to run a ROS program. Let's
test our installation by running two small ROS programs called `talker` and
`listener`. These two programs will send data back and forth using ROS to
perform the communication. One program was written in C++ and the other in
Python. Running these two different programs is a quick and easy way to check
that your ROS system is configured correctly. To start the talker run the following command.

``` {.sourceCode .bash}
source /opt/ros/eloquent/setup.bash
ros2 run demo_nodes_cpp talker
```

If everything is working correctly you should see something like the following:

``` {.sourceCode .bash}
kscottz@kscottz-ratnest:~$ ros2 run demo_nodes_cpp talker
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
[INFO] [talker]: Publishing: 'Hello World: 3'
....
```

Now, let's fire up the listener. We're going to use a Python listener in this
example to make sure we installed Python correctly. First we will need a second terminal. We can
open a new terminal tab by entering `CTRL-SHIFT-T` in our terminal. We can also
create a wholly new terminal by pressing `CTRL-ALT-T`. Pick whatever works best
for you. Now in your new terminal source your bash file and run the following
command.


``` {.sourceCode .bash}
source /opt/ros/eloquent/setup.bash
ros2 run demo_nodes_py listener
```

If everything is working correctly you should see something like the following:


``` {.sourceCode .bash}
kscottz@kscottz-ratnest:~$ ros2 run demo_nodes_py listener
[INFO] [listener]: I heard: [Hello World: 264]
[INFO] [listener]: I heard: [Hello World: 265]
[INFO] [listener]: I heard: [Hello World: 266]
```

Now that we have tested our ROS installation we can stop these two programs. In
ROS most programs run in infinite loops until the robot is shut down. To stop
these programs we navigate to the terminal running the program and press the
`Ctrl` and `C` keys simultaneously. We call this combo `CTRL-C` and you can use
it to stop just about any program in a terminal. Use it to stop the talk and
listener programs.
