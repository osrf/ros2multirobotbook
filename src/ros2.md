# An Introduction to ROS 2

In this chapter we will cover the basics of Robot Operating System (ROS) and
give you all the tools you need to build, debug, and understand robotic
applications. This chapter is laid out from the most general concepts, necessary
for decision makers to make sound decisions, to specific API references needed
by engineers to develop new robotic applications. Somewhere in between high
level concepts, and low level API commands lives the knowledge necessary for
those maintaining and supporting multi-robot deployments in the field. 

A good analogy to learning about ROS is the process of learning about motor
vehicles. At the practical, day-to-day level, most people will learn how to
start a vehicle and safely use it on a motorway. For these individuals, learning
about the high level concepts behind ROS, along with application-specific
commands is probably sufficient. Those who enjoy driving often choose to learn
how to repair and maintain their vehicle. If this is your interest level we
recommend learning the basics of the ROS command line interface. This will allow
you to "check the oil" of your robotic systems and make sure everything is
functioning correctly. Finally, if you are the type that would like to swap out
the engine of your vehicle with something more powerful, or potentially build a
wholly new vehicle from scratch, then the ROS API is the set of tools that will
make this possible. Generally speaking, automotive engineers don't appear into
the world fully formed, and the same is true for robotocists. It is advisable to
work through each phase of understanding as you develop your skills with ROS.  

Following from our analogy above the process of learning how to use robotic
systems built on ROS can be divided roughly into four parts. This chapter works
through these four parts of the process, using ROS 2. Subsequent chapters then
build upon this knowledge and discuss the subtleties of specific
applications. The four parts of this chapter are as follows.

* Meta-discussion of the tools and resources available to help you in the
  learning process.
  
* A high level discussion to the design patterns use in ROS. These patterns are
  roughly analogous to the subsystems you would find in a vehicle (engine,
  breaks, safety, climate control, etc). 
  
* A treatment of the command line interface (CLI) for ROS. The CLI is a set of
  programs for starting, inspecting, controlling, and monitoring a ROS
  robot. You can think of this topic as teaching you how check a robot's oil,
  and read the instrument panel. 

* An introduction to the ROS application programming interface. This section
  will show you how to create your own applications and modify existing software
  for your specific application. 
  
While this books aims to cover the basics it should be made clear that ROS, like
almost all software is a moving target. Technology moves quickly, and while
print media is helpful and delivering high fidelity instruction, that
instruction can become rapidly outdated. For this reason we start this chapter
with a meta-discussion of ROS resources that can be used to help you in your
learning process. 


{{#include ros2_tools_resources.md}}

{{#include ros2_design_patterns.md}}

{{#include ros2_api.md}}

{{#include ros2_cli.md}}


Pull from slide content below

Getting Help!




You should now be ready for the class!

Some Nomenclature as we Begin

-   **Package** -- A collection of code.
-   **Workspace** -- A workspace is a collection of source code / ROS
    packages that will run on a robot. It has a uniform directory
    structure. A good analogy is python virtual env, or "project" in
    most IDEs.
-   **Overlay** -- A second workspace with more/different/new packages.
    If there are multiple versions of a package/code then the one at the
    bottom is used.
-   **Underlay** -- The workspace, underneath an overlay, we're aware
    this is confusing.
-   **Colcon** -- The ROS 2 build tool. Think of it as a layer above
    CMake/Make/SetupTools that helps these tools work together smoothly.

This is a bit confusing. You may ask yourself why we have our own build
tool. The short of it is that the ROS ecosystem consists of tens of
thousands of developers, working on thousands of packages, across a
handful of platforms, using multiple languages. We needed a flexible
system to build code and one didn't exist at the time, and still doesn't
exist.

Let's Get Started

As we're diving headfirst into ROS our first job is to checkout a
repository of examples and build it. Roughly the steps to do this are as
follows.

-   Fire up a terminal manager inside the container. I use byobu. You
    can use whatever you want. You can also fire up 3 real terminals and
    call ade enter on them.
-   Source the ROS setup.bash file so we have the right version of ROS
    in our path.
-   Make a workspace called ros2\_example\_ws. We usually use \_ws to
    indicate a workspace.
-   Clone an example repository and change to the dashing branch.
    -   Generally ROS repos have a branch per release.
-   Use Colcon to build the source.

``` {.sourceCode .bash}
source /opt/ros/dashing/setup.bash
mkdir -p ~/ros2_example_ws/src
cd ~/ros2_example_ws
git clone https://github.com/ros2/examples src/examples
cd ~/ros2_example_ws/src/examples/
git checkout dashing 
cd ~/ros2_example_ws
colcon build --symlink-install
```

Nodes and Publishers

-   The core of ROS is the ROS pub/sub bus. In ROS parlance this is
    called topic.
    -   A topic has a message type that is published on the bus. These
        messages are defined in a yaml file and define the
        serialization/deserialization format for ROS messages.
    -   ROS has a lot of built in message types. There are lots of
        pre-defined messages for controlling a robot, distributing
        sensor data, and understanding the geometry of your robot.
    -   ROS publishers produce messages and slowly or as quickly as they
        need to.
    -   A ROS subscriber, subscribes to a topic and then does things
        with the information.
-   ROS has lots of built-in tools for managing topics. You can list
    them, echo (watch) them, rename them (called remap), and store them
    to file (called bagging).
-   ROS Nodes are basically programs, or processes that run concurrently
    on ROS.
    -   A ROS node can publish to one or more topics.
    -   That same node can subscribe to other topics.
    -   Many nodes subscribe to topics, process the data, and publish
        the results.
    -   ROS has tooling to start and stop multiple nodes at the same
        time.

Preparing to Run a ROS Node

-   Open a new terminal, in Byobu you can do this by pressing F2.
-   First we need to source the setup.bash file for our workspace. This
    will help ROS find the programs we built.
    -   source ./ros2\_example\_ws/install/setup.bash
    -   Protip: you can find any file using
        find ./ -name &lt;file name&gt;
-   **ROS Best Practice** *ALWAYS* build and execute in different
    terminals.
    -   The build terminal should source the global ROS setup.bash file
        (i.e. /opt/ros/dashing/setup.bash).
    -   The execution terminal should source the setup.bash of your
        workspace
    -   This is a common failure mode for new users. If something seems
        weird or funky. Create a new terminal and source the correct
        bash file.

Let's Run a Simple C++ Publisher Node.

-   ROS has an advanced, and fairly complex CLI interface. We'll cover
    it in depth in our next lesson.
-   We are going to ask ros to run the EXECUTABLE publisher\_lambda in
    our WORKSPACE named examples\_rclcpp\_minimal\_publisher.
-   The syntax for doing this is
    ros2 run &lt;WORKSPACE&gt; &lt;EXECUTABLE&gt;
-   To run our publishing node, let's run the following command in our
    execution terminal:
    ros2 run examples\_rclcpp\_minimal\_publisher publisher\_lambda
-   If everything works you should see something like this:

``` {.sourceCode .bash}
kscottz@ade:~$ ros2 run examples_rclcpp_minimal_publisher publisher_lambda 
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 0'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 1'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 2'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 3'
...
```

-   To exit the program press CTRL-C

What just happened?

-   We just executed a ROS node that publishes a simple string message
    to a topic called /topic twice a second.
-   I'll show you how I know this with some tools. We'll cover these
    tools in detail next time.

``` {.sourceCode .bash}
kscottz@ade:~$ ros2 topic list
/parameter_events
/rosout
/topic
kscottz@ade:~$ ros2 topic echo /topic
data: Hello, lambda world! 63
---
data: Hello, lambda world! 64
---
data: Hello, lambda world! 65
---
kscottz@ade:~$ ros2 topic hz /topic 
average rate: 2.000
min: 0.500s max: 0.500s std dev: 0.00011s window: 4
kscottz@ade:~$ 
```

Digging into the Code

-   Let's take a look at the code. Like a lot of software there is more
    than one way to skin a cat. Let's look at the member function
    approach.
-   Using your favorite editor open the following source file,
    ./ros2\_example\_ws/src/examples/rclcpp/minimal\_publisher/member\_function.cpp
-   **rclcpp** is an abbreviation of "ROS Client Library C++", its the
    ROS C++ API

``` {.sourceCode .c++}
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp" // THIS the header file for ROS 2 C++ API
#include "std_msgs/msg/string.hpp" // This is header for the messages we
                                   // want to user
                                   // These are usually auto generated. 

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
  // Make a class called Minimal Publisher
  class MinimalPublisher : public rclcpp::Node
  // Have it inherit from the ROS Node Class
```

Let's Build our Node's Constructor

-   The MinimalPublisher constructor inherits from the RCLCPP Base
    Class, gives the name a node, and sets our counter.
-   The next line creates a publisher object that publishes
    std\_msgs::msg.
-   The constructor then creates a callback to the function
    timer\_callback that gets called every 500ms.

``` {.sourceCode .c++}
class MinimalPublisher : public rclcpp::Node // Inherit from ROS Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0) // Set the node name
    {   //Create a publisher that pushes std_msgs::msg to the topic "topic" 
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); 
      timer_ = this->create_wall_timer( // Call timer_callback every 500ms
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
```

Now to Handle the Callback

-   In the callback function we do the following:
    -   Create the ROS std\_msgs::msg::String() to send to our topic.
    -   Construct the message that will be pushed to the ROS Topic
    -   Log the results.
    -   Actually publish the newly constructed message.

``` {.sourceCode .c++}
private:
void timer_callback()
{
  auto message = std_msgs::msg::String(); // create message
  message.data = "Hello, world! " + std::to_string(count_++); // Fill it up
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // Log it
  publisher_->publish(message); // Publish
}
// Create our private member variables. 
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
```

Finally, Let's Create the Main for our Node

-   This last little bit creates the main node entry point.
-   Initializes rcpcpp with the values from the command line.
-   Run's the MinimalPublisher, until a terminate is given
-   Finally the node cleans up everything and exits.

``` {.sourceCode .c++}
int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Init RCL
  rclcpp::spin(std::make_shared<MinimalPublisher>());// Run the minimal publish
  rclcpp::shutdown(); // Cleanup on shut down.
  return 0;
}
```

Exercise: Modify and Build this Node

-   Let's try to make a few modification to our node for practice.
    -   Make it run at 10Hz (100ms) instead of 500.
    -   Change the topic name from "topic" to "greetings."
    -   Change the message "Hello Open Road."
    -   Change the node name from minimal\_publisher,
        revenge\_of\_minimal\_publisher
-   Once you make these changes
    -   Save the file.
    -   Toggle over to your execution window run
    -   Run colcon build
    -   In your execution window run
        ros2 run examples\_rclcpp\_minimal\_publisher publisher\_member\_function

Let's Try Subscribing.

-   The pattern here is similar to publishing.
-   We basically inherit from the Node class, and define the topic and
    message we want.
-   Whenever that topic is published we hit a callback.
-   If everything is correctly configured the file is at
    -   /ros2\_example\_ws/src/examples/rclcpp/minimal\_subscriber/member\_function.cpp

``` {.sourceCode .c++}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
// Again we inherit the public interface of a ROS node. 
class MinimalSubscriber : public rclcpp::Node
{
  public:
  MinimalSubscriber() // Construct our node, calling it minimal_subscriber
  : Node("minimal_subscriber")
  { // Create a subscription, to messages of the format stdmsg:msg:String
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    // Subscribe to the topic, "topic" and set a callback for when things are pub'd 
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
  ...
```

More Subscriber

-   The subscriber node looks fairly similar to our publisher but
    instead of publishing on a regular callback, we get a callback when
    a new message hits our topic.

``` {.sourceCode .c++}
private:
  // Whenever we get a new messaged published on our topic
  // this callback will be executed.    
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    // Log the message that we are subscribed to 
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// This is effectively the same boiler plate from last time. 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

Let's Modify the Subscriber

-   In the publisher we changed the name of our publisher topic to
    greetings.
-   Let's change the subscribed topic to greetings.
-   Note that there are a lot of ways to change topic names, modifying
    source is just one approach. Often we just remap topics instead of
    changing source.
-   Once you have modified the subscriber run colocon build (it will
    build everything)
-   Open another terminal, source the bash file, and start the
    publisher.
    -   ros2 run examples\_rclcpp\_minimal\_publisher publisher\_member\_function
-   Now run our subscriber.
    -   ros2 run examples\_rclcpp\_minimal\_subscriber subscriber\_member\_function

The Result

If everything went well you should have two screens. The first screen
with the publisher should be spitting out the following

``` {.sourceCode .bash}
[INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1000'
[INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1001'
[INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1002'
[INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1003'
[INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1004'
```

The subscriber screen should be pushing out:

``` {.sourceCode .bash}
[INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1000'
[INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1001'
[INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1002'
[INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1003'
[INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1004'
```

**You can terminate both of these programs with CTRL-C**

*Congratulations, you now know the three most important ROS components,
nodes, publishers, and subscribers.*

Making Things Happen with Services

-   Publishing and subscribing nodes are the bread and butter of ROS.
    This pattern is great for moving around a lot of data, and
    processing it quickly.
-   However, we often want our robots to respond to data. To construct
    simple behaviors in ROS we use services.
-   A service is a robotic task that can be performed *synchronously*,
    which is just a fancy word for, "while you wait".
-   A good analogy for services would be a regular old function call. In
    most programs when you call a function, the code making the call
    waits for the function to return before proceeding.
-   A few toy examples of services for autonomous driving would be:
    -   Turning Lights Off/On.
    -   Checking a sensor and returning the results.
    -   Lock / Unlock a door or window.
    -   Beeping a horn.
-   Services can be called via the command line or through an API call
    within another node.
-   In ROS services are hosted within a ROS Node, and they can co-exist
    with other services as well as publishers and subscribers.

C++ Service Example

-   As a toy example of a ROS service we are going to make a node that
    offers an "AddTwoInts" service.
-   What will happen is the service has two inputs, and returns a single
    output.
-   There is a full tutorial [about the process
    here](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/).
    It goes into more detail and it is worth looking at.

Let's start by looking at a prebuilt srv file for this tutorial. If you
were writing this service from scratch you would need to build this srv
file yourself, but for this example there is one ready for us already.
We'll use less to peek into the srv file.

Run the following:
less /opt/ros/dashing/share/example\_interfaces/srv/AddTwoInts.srv

The file should have the following:

``` {.sourceCode .yaml}
int64 a     # <== An input, of type int64, called a 
int64 b     # <== An input, of type int64, called b
---
int64 sum   # <== An output, of type int64, called sum
```

Defining A Service

Essentially our service is a remote procedure call of a function that
looks like this in pseudocode: int64 sum = AddTwoInts(int64 a, int64b);.

Let's take a look at the C++ code that defines the service. Use your
favorite text editor to open the following file:
./ros2\_example\_ws/src/examples/rclcpp/minimal\_service/main.cpp.

``` {.sourceCode .C++}
// This hpp file is autogenerated from the srv file. 
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"// ROS header.
// Scope resolution to our services. 
using AddTwoInts = example_interfaces::srv::AddTwoInts;
// shared pointer to logger
rclcpp::Node::SharedPtr g_node = nullptr;
// Perform the service call 
void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,// Header with timestamp etc 
  const std::shared_ptr<AddTwoInts::Request> request,    // This is the input, two int64 a,b
  const std::shared_ptr<AddTwoInts::Response> response)  // This response is int64 sum
{
  (void)request_header;
  RCLCPP_INFO( // Logger message. 
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
 response->sum = request->a + request->b; // the actual function. 
```

> }

ROS 2 Service Main

``` {.sourceCode .C++}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // get global ROS pointer
  g_node = rclcpp::Node::make_shared("minimal_service"); 
  // Create a service, of type AddTwoInts, named add_two_ints, that points to handle_service
  auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
  rclcpp::spin(g_node); // run until shutdown
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
```

The main entry point is pretty simple. It does the following.

-   Initialize the program.
-   Get a shared pointer to the ROS node interface.
-   Create the service, of type AddTwoInts, named add\_two\_ints,
    pointing to the function handle\_service.
-   Run the node until shutdown.

Let's Build and Run our Service

First we will fire up our service! The syntax for this is
ros2 run &lt;pkg&gt; &lt;program&gt;.

``` {.sourceCode .bash}
kscottz@ade:~$ ros2 run examples_rclcpp_minimal_service service_main 
```

At this point nothing should happen. We need to *call* the service. To
do that we'll use a command line tool that's a little... long.

We'll talk about this more in the next lesson, but the syntax is
roughly,
ros2 service call &lt;service\_name&gt; &lt;service\_call\_format&gt; &lt;actual\_data&gt;.

In this case our service name is /add\_two\_ints and the data type can
be found in example\_interfaces/AddTwoInts, and the input is yaml
encased in quotation marks. Move over to a new terminal and enter the
following:

``` {.sourceCode .bash}
kscottz@ade:~/ros2_example_ws$ ros2 service call /add_two_ints example_interfaces/AddTwoInts "{a: 1, b: 1}"
waiting for service to become available...
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=1, b=1)

response:
example_interfaces.srv.AddTwoInts_Response(sum=2)
```

Now switch back to your original terminal, you should see something like
this:

``` {.sourceCode .bash}
kscottz@ade:~$ ros2 run examples_rclcpp_minimal_client client_main  
3[INFO] [minimal_service]: Incoming request
a: 1 b: 1
```

Congratulations, you just made your first service call!

Using a Service in Code

We just called our service from the command line to test it, but more
often than not we would want to do this in source code.

Let's look at an example of how to do that. In your editor or using less
take a look at the following file:
/home/kscottz/ros2\_example\_ws/src/examples/rclcpp/minimal\_client/main.cpp

``` {.sourceCode .C++}
// snipped 
#include "example_interfaces/srv/add_two_ints.hpp" // include the service header file. 
#include "rclcpp/rclcpp.hpp"
// Scope resolution on underlying call signature. 
using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv); //  init ROS C++ interface. 
   auto node = rclcpp::Node::make_shared("minimal_client"); // shared node memory.  
   auto client = node->create_client<AddTwoInts>("add_two_ints"); // create client interface.
   while (!client->wait_for_service(std::chrono::seconds(1))) {// poll for service to come online
     if (!rclcpp::ok()) {  // if service doesn't come online, exit gracefully                    
       RCLCPP_ERROR(node->get_logger(), 
       "client interrupted while waiting for service to appear.");
   return 1;
   }
   RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
}
```

C++ Service Client Part Deux

``` {.sourceCode .C++}
// shared memory to request 
auto request = std::make_shared<AddTwoInts::Request>(); 
request->a = 41;  // set the input values
request->b = 1;   // set the input values
auto result_future = client->async_send_request(request); // Send the request
if (rclcpp::spin_until_future_complete(node, result_future) != // spin until result
   rclcpp::executor::FutureReturnCode::SUCCESS)
{
   RCLCPP_ERROR(node->get_logger(), "service call failed :("); 
   return 1;
}
auto result = result_future.get(); // Get the result 
RCLCPP_INFO(node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
   request->a, request->b, result->sum); // print the result
```

> rclcpp::shutdown(); // shutdown return 0; }

Let's Run Our Client

-   Now we're going to run our service and then call it from the client.
-   You'll need two terminals to do this. Remember F2/F3 let you open
    and switch to a new terminal in ADE.

First fire up your service if it isn't already running.

``` {.sourceCode .bash}
$ ros2 run examples_rclcpp_minimal_service service_main 
```

Now start the client in a second terminal.

``` {.sourceCode .bash}
$ ros2 run examples_rclcpp_minimal_client client_main 
[INFO] [minimal_client]: Result of add_two_ints: for 41 + 1 = 42
```

The client should fire off a request right away. You can see the result.

Finally, toggle back to the service.

``` {.sourceCode .bash}
$ ~/ros2_example_ws$ ros2 run examples_rclcpp_minimal_service service_main 
[INFO] [minimal_service]: Incoming request
a: 41 b: 1
```

You can see the debug input has been printed to the terminal.

ROS C++ Actions

-   Actions are ROS / ROS 2's answers to asynchronous remote procedure
    calls.
-   Notice how quickly how fast our service call happened. It was more
    or less instant.
-   Actions are the preferred approach for things that may not happen
    instantaneously.
-   The canonical example of a ROS Action would be sending the robot a
    command to navigate to a way point.
-   The process of navigation is going to take a bit of time, what we
    want to do is to kick off the process, wait for updates, and then
    once things are complete we get a result.
-   Just like services there are two parts of an action. The action
    server and the action client. *Note that there can be more than one
    client.*
-   Actions become fairly complex as they can serve multiple clients.
    This means the action may need to keep track of multiple concurrent
    connections.
    -   Since action servers can get overwhelmed by requests, they need
        to *accept* every request before proceeding to process it.
    -   The clients can also *cancel* at any time, so that needs to be
        handled.

Parts of an Action

-   Find the action. An action server may be down!
-   The Action Request -- the service *can* decline to take an action.
-   The Action being accepted.
-   The Action being canceled. Sometimes the client changes its mind.
-   The action "feedback", sending back info from server to client.
-   Send the result to the client -- the result could be the thing
    happened succesfully, or not!

Fibonacci Action

For our action server we're going to create a toy example, this example
will calculate the Nth number in the [Fibonacci
series](https://en.wikipedia.org/wiki/Fibonacci_number). So, what will
happen when we call this toy action?

-   We will call the action with a single integer indicating the
    sequence number of the Fibonacci number we want.
-   The action will update us as it calculates the sequence of numbers
    and update it us as it calculates a new one.
-   When the action gets to our desired number in the sequence, it will
    return the results.
-   For example, if we called action with the input 7, we would get the
    seventh Fibonacci number. Which means, given the series &lt;0, 1, 1,
    2, 3, 5, 8&gt;, would be the number 8.
-   The action should update us along the way in the calculation. It
    should return the series of numbers every time it calculates a new
    number.

Action Definition Files

-   Actions use a definition file to build all of the ROS boiler plate
    like cross language header/definition files for use in multiple
    programming languages.
-   These action files are written in YAML and use the \*.action suffix.
-   The ROS meta build system colcon will use these action files to
    auto-magically generate all of the header files.

Let's take a look at an action file.

``` {.sourceCode .bash}
/opt/ros/dashing/share/example_interfaces/action/Fibonacci.action
# Goal -- the input, the order we want like 7
int32 order 
---
# Result -- the *final result*, here the list of values 0,1,1,2,3,5,8....
int32[] sequence
---
# Feedback -- the *intermediate result* so <0>,<0,1>,<0,1,1>,<0,1,1,2> ...
int32[] sequence
Fibonacci.action (END)
```

Really quick, let's look under the hood!

As we said previously, the \*.action is used to auto-generate a bunch of
other files. We can see this if we go down one directory to msg.

What we'll see is that the \*.action file is used to generate a bunch of
ROS topic messages mapping to states in our action.

Essentially a ROS action is built upon ROS nodes and ROS topics

``` {.sourceCode .bash}
kscottz@ade:/opt/ros/dashing/share/example_interfaces/action/msg$ cd ~/
kscottz@ade:~$ cd /opt/ros/dashing/share/example_interfaces/action/msg/
kscottz@ade:/opt/ros/dashing/share/example_interfaces/action/msg$ ls
FibonacciActionFeedback.msg  FibonacciAction.msg        FibonacciFeedback.msg  FibonacciResult.msg
FibonacciActionGoal.msg      FibonacciActionResult.msg  FibonacciGoal.msg
kscottz@ade:/opt/ros/dashing/share/example_interfaces/action/msg$ less FibonacciActionGoal.msg
# This file is automatically generated by rosidl-generator
std_msgs/Header header
actionlib_msgs/GoalID goal_id
FibonacciGoal goal
FibonacciActionGoal.msg (END)
kscottz@ade:/opt/ros/dashing/share/example_interfaces/action/msg$ cd ~
```

Let's take a look at Action Server

-   Let's take a look at how our Fibonacci action server.
-   Use your favorite text editor to open:
    /home/kscottz/ros2\_example\_ws/src/examples/rclcpp/minimal\_action\_server/member\_functions.cpp

``` {.sourceCode .C++}
<headers cut>
class MinimalActionServer : public rclcpp::Node
{
public:  // Pre-defined interface files. 
   using Fibonacci = example_interfaces::action::Fibonacci;
   using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
   explicit MinimalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
   : Node("minimal_action_server", options)
   {
     using namespace std::placeholders;
 // SCARY call to define that this is a Fib. action and what functions
 // attach to what events in the action lifecycle. 
 this->action_server_ = rclcpp_action::create_server<Fibonacci>(
 this->get_node_base_interface(),      // The action server is basically
 this->get_node_clock_interface(),     // a node and we need return pointers
 this->get_node_logging_interface(),   // to all of standard interaces. 
 this->get_node_waitables_interface(),
 "fibonacci",  // and bind our member functions to topic events. 
 std::bind(&MinimalActionServer::handle_goal, this, _1, _2), 
 std::bind(&MinimalActionServer::handle_cancel, this, _1),
 std::bind(&MinimalActionServer::handle_accepted, this, _1));
   }
```

Actions: Accept or Cancel

Let's deal with accepting a goal, or canceling a goal.

``` {.sourceCode .C++}
private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
   const rclcpp_action::GoalUUID & uuid, // Each request gets a UUID 
   std::shared_ptr<const Fibonacci::Goal> goal) // The goal object
  {
     RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
 (void)uuid;
 // Let's reject sequences that are over 9000
 if (goal->order > 9000) {
   return rclcpp_action::GoalResponse::REJECT;
   } // respond with "yes, we'll process this request. 
   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
   const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
   RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
   (void)goal_handle;
   return rclcpp_action::CancelResponse::ACCEPT;
}
```

The Meat of the Fib Function

``` {.sourceCode .C++}
void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{ // This is the meaty part of the function 
   RCLCPP_INFO(this->get_logger(), "Executing goal");
   rclcpp::Rate loop_rate(1);
   const auto goal = goal_handle->get_goal(); // this is our goal value
   auto feedback = std::make_shared<Fibonacci::Feedback>(); // this is our feedback object
   auto & sequence = feedback->sequence; // this is our list of fib values. 
   sequence.push_back(0);
   sequence.push_back(1);
   auto result = std::make_shared<Fibonacci::Result>(); // This is the final result. 

   // Do fib as long as ROS is ok!
   for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
   // Check if there is a cancel request
   if (goal_handle->is_canceling()) { // Handle a cancel result!
     result->sequence = sequence;
     goal_handle->canceled(result);
     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
     return;
   }
   // Update sequence
   sequence.push_back(sequence[i] + sequence[i - 1]);
   // Publish feedback
   goal_handle->publish_feedback(feedback);
   RCLCPP_INFO(this->get_logger(), "Publish Feedback");

   loop_rate.sleep();
   }

 // Check if goal is done
 if (rclcpp::ok()) {
   result->sequence = sequence;
   goal_handle->succeed(result);
   RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
 }
```

> }
>
> void handle\_accepted(const
> std::shared\_ptr&lt;GoalHandleFibonacci&gt; goal\_handle) { // Our
> request was accepted, fire off a new thread. using namespace
> std::placeholders; // this needs to return quickly to avoid blocking
> the executor, so spin up a new thread
> std::thread{std::bind(&MinimalActionServer::execute, this, \_1),
> goal\_handle}.detach(); }

Let's Put our Class into an Executable

``` {.sourceCode .C++}
int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);

   auto action_server = std::make_shared<MinimalActionServer>();

   rclcpp::spin(action_server);

   rclcpp::shutdown();
   return 0;
}
```

Let's Run Our Action and Call It.

-   Ordinarily you would call colcon build in your workspace to build
    the source code. We're just inspecting this method so this isn't
    necessary.
-   We'll start the action server and then call it manually using the
    ROS 2 CLI.

``` {.sourceCode .bash}
kscottz@ade:~/ros2_example_ws$ ros2 run examples_rclcpp_minimal_action_server action_server_member_functions
```

Now we're going to manually call the server from the ROS 2 CLI. We'll
cover this in more depth in the next lesson. If you're using byobu use
F3 to go to a second terminal or F2 to make a new one.

``` {.sourceCode .bash}
$ ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci '{order: 10}'
Waiting for an action server to become available...
Sending goal:
   order: 10

Goal accepted with ID: 0c1b3779c7ea44b69d54c6e1cfac3ff6

Result:
   sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]

Goal finished with status: SUCCEEDED
```

Meanwhile, Back at The Server

You can use F3 to see what happened to our action and its status
updates.

``` {.sourceCode .bash}
[INFO] [minimal_action_server]: Received goal request
[INFO] [minimal_action_server]: Executing goal...
[INFO] [minimal_action_server]: Publishing feedback: array('i', [0, 1, 1])
[INFO] [minimal_action_server]: Publishing feedback: array('i', [0, 1, 1, 2])
[INFO] [minimal_action_server]: Publishing feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [minimal_action_server]: Publishing feedback: array('i', [0, 1, 1, 2, 3, 5])
... SNIP ... 
[INFO] [minimal_action_server]: Returning result: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
```

Action Client

-   Let's take at the client side API implementation. Open the file:

\~/ros2\_example\_ws/src/examples/rclcpp/ minimal\_action\_client/member\_functions.cpp

-   We'll address the basic implementation but that directory has
    additional examples for other use cases and things like canceling an
    action mid-process.
-   It is worth understanding what we're doing, it is more than sending
    just the goal. Roughly this class does the following:
    -   Check's for a connection to ROS, and the action server.
    -   Sends the goal.
    -   Checks that the goal was "accepted" after sending.
    -   Updates the log/screen as interim feedback gets sent.
    -   Receives the final results.

Let's Create A Client Class

``` {.sourceCode .C++}
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
class MinimalActionClient : public rclcpp::Node
{
   public: // looks familiar, pulling in the action interface, and the goal type
   using Fibonacci = example_interfaces::action::Fibonacci;
   using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

   explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
   : Node("minimal_action_client", node_options), goal_done_(false)
   {  // Create a client interface. 
      this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
  this->get_node_base_interface(),
  this->get_node_graph_interface(),
  this->get_node_logging_interface(),
  this->get_node_waitables_interface(),
  "fibonacci");
  // Create a time and have callback to send goal in 500ms
  this->timer_ = this->create_wall_timer(
  std::chrono::milliseconds(500),
  std::bind(&MinimalActionClient::send_goal, this));
   }
```

Sending the Goal

Our client constructor above set a time to call send\_goal after 500ms.
We'll bind our member functions to the action events and then send the
goals.

Handling the Responses

Next up we create our private member variables and define the functions
that get called with the goal response and the periodic feedback.

``` {.sourceCode .C++}
private:
   rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
   rclcpp::TimerBase::SharedPtr timer_;
   bool goal_done_;
   // handle the response to our request 
   void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
   {
      auto goal_handle = future.get();
  if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
   }
   // handle the feedback calls, these should be the format of feedback. 
   void feedack_callback(
   GoalHandleFibonacci::SharedPtr,
   const std::shared_ptr<const Fibonacci::Feedback> feedback)
   {
      RCLCPP_INFO(
  this->get_logger(),
  "Next number in sequence received: %" PRId64,
  feedback->sequence.back());
   }
```

Handling The Result

``` {.sourceCode .C++}
// handle result callback
void result_callback(const GoalHandleFibonacci::WrappedResult & result)
{
   this->goal_done_ = true;
   switch (result.code) {
     case rclcpp_action::ResultCode::SUCCEEDED:
       break;
 case rclcpp_action::ResultCode::ABORTED:
   RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
       return;
 case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
 default:
       RCLCPP_ERROR(this->get_logger(), "Unknown result code");
       return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(this->get_logger(), "%" PRId64, number);
    }
  }
};  // class MinimalActionClient
```

Running our Client Class

Finally the main function that attaches to our node class. It simply
creates a class instance and runs until completion.

``` {.sourceCode .C++}
int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);
   auto action_client = std::make_shared<MinimalActionClient>();

   while (!action_client->is_goal_done()) {
     rclcpp::spin_some(action_client);
   }

   rclcpp::shutdown();
   return 0;
}
```

Let's Run our Client

-   We'll start the action server the same way as before

``` {.sourceCode .bash}
kscottz@ade:~/ros2_example_ws$ ros2 run examples_rclcpp_minimal_action_server action_server_member_functions
```

Next we'll run our client.

``` {.sourceCode .bash}
kscottz@ade:~/ros2_example_ws$ ros2 run examples_rclcpp_minimal_action_client action_client_member_functions 
[INFO] [minimal_action_client]: Waiting for action server...
[INFO] [minimal_action_client]: Sending goal request...
[INFO] [minimal_action_client]: Goal accepted :)
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34])
[INFO] [minimal_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
[INFO] [minimal_action_client]: Goal succeeded! Result: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
```

Wrapping Up...

-   We've just seen the set of API primitives upon which most ROS
    systems are made.
-   Generally speaking, when you build a robot you work from simple to
    complex. You build the nodes and topics first, then the services,
    and finally the actions.
-   While we addressed all of these topics with the C++ API there is an
    equivalent Python API that works similarly.
-   Moreover, there are additional API primitives that you can check
    out.
-   All of these examples are in the workspace that we created.
-   I would encourage you to modify these examples to build a better
    idea of how they work.

**Next time we'll cover the ROS 2 CLI**
