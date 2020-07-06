# The ROS API

ROS comprises many software libraries that provide a wide array of
functionality useful when building robot applications. The libraries
you need will depend on the details of your project. In this section we will
introduce two core libraries that you are likely to interact with
frequently when developing with ROS:

- `rclpy` : Python client library
- `rclcpp` : C++ client library

A ROS *client library* provides the data structures, functions, and
syntactic sugar that make it convenient to develop in a particular
programming language. Here we will cover just the Python and C++
libraries because they're the most widely used. But you can find ROS
client libraries for many other languages, from Ada to JavaScript to
Rust, and beyond.

<div style="border: 1px; border-style: solid; padding: 1em">
<b>Note</b>: In this section we aim for a gentle and efficient introduction
to the ROS API. In service of that goal, we will purposefully ignore
and/or violate various conventions and patterns.
</div>

## Publishing and Subscribing to Topics in Python

Publishing data with ROS is easy. Here is a complete Python program that
publishes string messages:

``` {.sourceCode .py}
from time import sleep
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('my_publisher')
pub = node.create_publisher(String, 'chatter', 10)
msg = String()
i = 0
while rclpy.ok():
    msg.data = f'Hello World: {i}'
    i += 1
    print(f'Publishing: "{msg.data}"')
    pub.publish(msg)
    sleep(0.5)
```

Try it out yourself. (Make sure that in every shell used you have
sourced your ROS setup file as we discussed in the previous chapter; e.g., `source
/opt/ros/foxy/setup.bash`.) Copy the code block above into a file, call
it `talker.py`, then feed it to your Python3 interpreter:


``` {.sourceCode .bash}
$ python3 talker.py
```

You should see:
```
Publishing: "Hello world: 0"
Publishing: "Hello world: 1"
Publishing: "Hello world: 2"
```

It prints to console, but is the data going anywhere? We can check
our work using the `ros2 topic` tool that was introduced earlier. In
another shell (leave your talker running), run:

``` {.sourceCode .bash}
$ ros2 topic echo chatter
```

You should see the following, though the numbers will vary depending on timing between the two
commands:

```
data: 'Hello world: 13'
---
data: 'Hello world: 14'
---
data: 'Hello world: 15'
```

So we have a working talker. Now we can add our own listener to use in
place of `ros2 topic`. Here is a complete Python program that subscribes
to string messages and prints them to console:

``` {sourceCode .py}
import rclpy
from std_msgs.msg import String

def cb(msg):
    print(f'I heard: "{msg.data}"')

rclpy.init()
node = rclpy.create_node('my_subscriber')
sub = node.create_subscription(String, 'chatter', cb, 10)
rclpy.spin(node)
```

Try it out yourself. Copy the code block above into a file and call it
`listener.py`. With your talker still running in one shell, start up your
listener in another shell:

``` {sourceCode .bash}
$ python3 listener.py
```

You should see (again, numbers will vary depending on timing):

```
I heard: "Hello world: 35"
I heard: "Hello world: 36"
I heard: "Hello world: 37"
```

### Digging into the Python Code

Now that we know these programs work, we can dig into their code. Both
programs start with the same preamble:

``` {sourceCode .py}
import rclpy
from std_msgs.msg import String
```

We need to import the `rclpy` client library, which gives us
much of what we need to write ROS applications in Python. But we also need
to specifically import the ROS message type(s) that we will use. In this
case we are using the simple `std_msgs/String` message, which contains a
single field called `data`, of type `string`. If we wanted to use the
`sensor_msgs/Image` message, which represents camera images, then we
would write `from sensor_msgs.msg import Image`.

After the imports, both programs perform common initialization:

``` {sourceCode .py}
rclpy.init()
node = rclpy.create_node('my_node_name')
```

We initialize the `rclpy` library and then call into it to create a
`Node` object, giving it a name. Subsequently we will operate on that
`Node` object.

In the talker, we use the `Node` object to create a `Publisher` object:

``` {sourceCode .py}
pub = node.create_publisher(String, 'chatter', 10)
```

We declare the type of data we will publish (`std_msgs/String`), the
name of the topic on which we will publish (`chatter`), and the maximum
number of outbound messages to locally queue up (10). That last
argument comes into play when we are publishing faster than subscribers
are consuming the data.

The equivalent step in the listener is to create a `Subscription`
object:


``` {sourceCode .py}
sub = node.create_subscription(String, 'chatter', cb, 10)
```

The type (`String`) and topic name (`chatter`) arguments have the same meaning
as the `create_publisher()` call, and the final argument (10) is
setting an analogous maximum queue size for inbound messages. The key
difference is the `cb` argument, which refers to this *callback*
function that we also defined in the listener:

``` {sourceCode .py}
def cb(msg):
    print(f'I heard: "{msg.data}"')
```

That function will be called whenever the listener receives a message,
and the received message will be passed in as an argument. In this case
we simply print the content to console.

With the callback defined and the `Subscription` created, the rest of
the listener is one line:

``` {sourceCode .py}
rclpy.spin(node)
```

This call hands control over to `rclpy` to wait for new messages to
arrive (and more generally for events to occur) and invoke our callback.

Back in the talker, we create a simple loop to use our `Publisher`:

``` {sourceCode .py}
msg = String()
i = 0
while rclpy.ok():
    msg.data = f'Hello World: {i}'
    i += 1
    print(f'Publishing: "{msg.data}"')
    pub.publish(msg)
    sleep(0.5)
```

These steps are clear enough: we create a message object and then on
each iteration of the loop, we update the message content and publish
it, sleeping briefly between iterations.

## Publishing and Subscribing to Topics in C++

Now we will write the same talker and listener pair, this time in C++.

Here is a complete C++ program that publishes string messages:

``` {sourceCode .cpp}
#include <unistd.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
  std_msgs::msg::String message;
  auto i = 0;
  while (rclcpp::ok()) {
    message.data = "Hello world: " + std::to_string(i++);
    std::cout << "Publishing: " << message.data << std::endl;
    pub->publish(message);
    usleep(500000);
  }
  return 0;
}
```

Of course, as for all C++, we need to compile this program. Managing the
compilation arguments for C++ is cumbersome, so we use CMake to help.
Here is the complete CMake code that allows us to build the talker
example:

```
cmake_minimum_required(VERSION 3.5)
project(talker_listener)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker talker.cpp)
target_include_directories(talker PRIVATE ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
```

Try it out yourself. Copy the C++ code into a file called `talker.cpp`
and the CMake code into a file called `CMakeLists.txt`. Have them
side-by-side in a directory and then invoke `cmake` followed by `make`:


``` {sourceCode .bash}
$ cmake .
$ make
```

You should end up with a compiled executable called `talker`. Run it:


``` {sourceCode .bash}
$ ./talker
```

You should see:
```
Publishing: "Hello world: 0"
Publishing: "Hello world: 1"
Publishing: "Hello world: 2"
```

Keep the talker running and another shell try `ros2 topic` to listen
in:

``` {.sourceCode .bash}
$ ros2 topic echo chatter
```

You should see (numbers will vary depending on timing between the two
commands):
```
data: 'Hello world: 13'
---
data: 'Hello world: 14'
---
data: 'Hello world: 15'
```

Now we can write our own listener to use in place of `ros2 topic`. Here
is a complete C++ program that subscribes to string messages and prints
them to console:

``` {sourceCode .cpp}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::cout << "I heard: " << msg->data << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_subscriber");
  auto sub = node->create_subscription<std_msgs::msg::String>("chatter", 10, cb);
  rclcpp::spin(node);
  return 0;
}
```

Copy the code block into a file called `talker.cpp`. To arrange for it
to be compiled, we also need to add some corresponding CMake code to the
bottom of our `CMakeLists.txt` file from earlier:

```
add_executable(listener listener.cpp)
target_include_directories(listener PRIVATE ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
```

Configure and build again:

``` {sourceCode .bash}
$ cmake .
$ make
```

Now you should have also have a `listener` executable. With your talker
still running in one shell, start up your listener in another shell:


``` {sourceCode .bash}
$ ./listener
```

You should see (again, numbers will vary depending on timing):
```
I heard: "Hello world: 35"
I heard: "Hello world: 36"
I heard: "Hello world: 37"
```

### Digging into the C++ Code

Now that we know these programs work, we can dig into their code. Both
programs start with the same preamble:

``` {sourceCode .cpp}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
```

We always need to include the `rclcpp` client library, which gives us
much of what we need to write ROS applications in C++. But we also need to
specifically import the ROS message type(s) that we will use. In this
case we are using the simple `std_msgs/String` message, which contains a
single field called `data`, of type `string`. If we wanted to use the
`sensor_msgs/Image` message, which represents camera images, then we
would `#include "sensor_msgs/msg/image.hpp"`.

After the imports, both programs perform common initialization:

``` {sourceCode .cpp}
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_node_name");
```

We initialize the `rclcpp` library and then call into it to create a
`Node` object, giving it a name. Subsequently we will operate on that
`Node` object.

In the talker, we use the `Node` object to create a `Publisher` object:

``` {sourceCode .cpp}
  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
```

We declare via template the type of data we will publish (`std_msgs/String`), the
name of the topic on which we will publish (`chatter`), and the maximum
number of outbound messages to locally queue up (10). That last
argument comes into play when we are publishing faster than subscribers
are consuming the data.

The equivalent step in the listener is to create a `Subscription`
object:


``` {sourceCode .cpp}
  auto sub = node->create_subscription<std_msgs::msg::String>("chatter", 10, cb);
```

The type (`String`) and topic name (`chatter`) arguments have the same meaning
as for the `create_publisher()` call, and the numerical argument (10) is
setting an analogous maximum queue size for inbound messages. The key
difference is the `cb` argument, which refers to this *callback*
function that we also defined in the listener:

``` {sourceCode .cpp}
void cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::cout << "I heard: " << msg->data << std::endl;
}
```

That function will be called whenever the listener receives a message,
and the received message will be passed in as an argument. In this case
we simply print the content to console.

With the callback defined and the `Subscription` created, the rest of
the listener is one line:

``` {sourceCode .cpp}
  rclcpp::spin(node);
```

This call hands control over to `rclcpp` to wait for new messages to
arrive (and more generally for events to occur) and invoke our callback.

Back in the talker, we create a simple loop to use our `Publisher`:

``` {sourceCode .cpp}
  std_msgs::msg::String message;
  auto i = 0;
  while (rclcpp::ok()) {
    message.data = "Hello world: " + std::to_string(i++);
    std::cout << "Publishing: " << message.data << std::endl;
    pub->publish(message);
    usleep(500000);
  }
```

In these steps we create a message object, and then on
each iteration of the loop we update the message content and publish
it, sleeping briefly between iterations.

## Where to Go From Here

That was a very brief introduction and we only covered topics, not
services, actions, parameters, or the many other facets of ROS. Luckily,
the online [ROS tutorials](https://index.ros.org/doc/ros2/Tutorials) are
an excellent resource for learning about the rest of ROS. We
specifically recommend the [Beginner: Client
Libraries](https://index.ros.org/doc/ros2/Tutorials/#beginner-client-libraries)
collection as a natural next step after reading this chapter.

## Regarding the Shortcuts

In this section we have presented the simplest, shortest example ROS
programs that we could come up with. Such programs are easy to
understand and learn from, as they do not have unnecessary structure or
decoration. But in exchange such programs are not easily extensible,
composable, or maintainable.

The techniques that we used in the example code in this section are
useful for prototyping and experimentation (an important aspect of any
good robotics project!), but we do not recommend them for serious work.
As you go through the [ROS
tutorials](https://index.ros.org/doc/ros2/Tutorials) and start reading
existing ROS code, you will learn about a number of concepts, patterns,
and conventions, such as:

- organizing your code into *packages*
- organizing your packages into a *workspace*
- managing *dependencies* among packages
- using the `colcon` tool to build code in multiple packages in dependency order
- using the `ament` module in your `CMakeLists.txt` files
- structuring your code to allow run-time control of how nodes maps to processes
- using the client libraries' console-logging routines for output to screen and elsewhere

These techniques will serve you well when you start building your own ROS
applications, especially when you want to share your code with others, whether
on your team or out in the world.
