# UI

## Introduction

We have looked at how we can integrate new hardware into RoMi, now we will look at how we can write end user applications that integrates into RoMi. We will briefly go through the requirements of a RoMi UI application and proceed with a tutorial of how we can write a React Webapp that interfaces with RoMi.

## Overview

The most common way to write a UI application is to make use of a UI framework, there are many UI frameworks out there, here are some of the popular frameworks out there

  * Multi-Paradigm
    * Qt
    * React native
    * Xamarin
    * Flutter
  * Web
    * React
    * Angular
    * Vue
  * Desktop
    * wxWidgets
    * Gtk
    * WPF (Windows only)

Each of these frameworks has its own pros and cons, there isn't a "best" framework for all use cases. That being said, some frameworks are more suitable to integrate with RoMi due to the technologies they are built upon, to understand why, let's take a brief overview at how a RoMi UI application would work.

## How does a RoMi UI Application Work?

Imagine a simple UI application with a label that shows the current status of a door and a button that opens/closes the door. Recall that in the door adapter, we publishes door states and subscribes to door requests, here we are doing the reverse, subscribing to door states and publishing door requests. We will be listening to door states coming in from RoMi and updating our display to match, and sending door requests when the user clicks the open or close button.

We need to use a libray to help us do that, in the hardware adapter examples, we are using the `rclcpp` library, but `rclcpp` isn't the only way to use ROS2, here are some libraries that also works well

  * Direct
    * rcl (C)
    * rclcpp (C++)
    * rclpy (python)
    * rclnodejs (nodejs)
  * Indirect
    * soss (websocket)
    * rosbridge (websocket)

A "direct" library is able to send and receive ROS2 messages directly from the application, in contrast, an indirect library uses a middleman service to forward ROS2 messages. In general, the "direct" approach is more preferable but sometimes it is not possible on the target platform. In these cases, the indirect approach can be used.

<div style="border: 1px; border-style: solid; padding: 1em">
<b>Note</b>: This is not an exhaustive list, you can find many third party projects that aims to bring the ROS2 ecosystem to more platforms, the <code>ros2-dotnet</code> project for example, provides C# bindings for ROS2. You can also write your own bindings and middlewares!
</div>

At the end of the day, writing a RoMi UI application isn't much different from writing any other UI application, the only difference being that we will be sending/receiving ROS2 in response to user input and updating the state.

# Tutorial: React Webapp

In this section, we will go through an example of creating a React based webapp to monitor door states and send door open/close requests. The tutorial will focus on the aspects of communicating with RoMi, basic knowledge of React and typescript is needed.

Note that this isn't the only way to create a RoMi UI application, as mentioned before, you can use any UI toolkit, the only requirement is to be able to send/receive ROS2 messages.

## Requirements

  * nodejs >= 10
  * soss (https://github.com/osrf/soss)
  * soss-romi-plugins (https://github.com/osrf/soss-romi-plugins)
  * rmf_demo (https://github.com/osrf/rmf_demo)

We will not go through the process of setting up the dependencies, instructions to set them up can be easily found on the web or from their project homepages.

## Setting Up

We will be using an example from rmf_demo as the RoMi deployment that we will be interacting with. This is a good time to test if your installation of `rmf_demo` is working properly, start the demo with the following command

```
ros2 launch demos office.launch.xml
```

Next let's test if soss is working. In order to run soss, you need to provide it with a configuration file, you can use [this](react-app-tutorial/soss.yaml) template as a start. You would need a certificate for soss, refer to various tutorials online to generate one, if you are using a self-signed cert, also make sure your browser is set to accept it for websocket connections. Add the path of your cert and key to the soss config and try starting soss with

```
soss <path_to_config>
```

Once the environment is set up, we can finally proceed to setting up our actual app. First, create a React workspace with

```
npx create-react-app react-app-tutorial --template typescript
```

Go inside the newly created `react-app-tutorial` directory and run the follow command to install all the javascript dependencies we will be using

```
npm install @osrf/romi-js-core-interfaces @osrf/romi-js-soss-transport
```

These libraries are not strictly required but they contain helpful functions to use soss and to communicate with RoMi. If you are building a javascript based RoMi app, it is recommended to make use of them, we will see later how they greatly simplifies the communicaton to RoMi.

<div style="border: 1px; border-style: solid; padding: 1em">
<b>Note</b>: Other than <code>@osrf/romi-js-soss-transport</code>, there is also <code>@osrf/romi-js-rclnodejs-transport</code> which is able to send ROS2 messages directly, however it does not work on the browser. It is preferred if you are writing a nodejs based desktop application using something like electron, or you are writing a server based application like a REST API provider.
</div>

## Door Component

First of all, let's create a simple component to show a door's state along with open and close buttons. Create a new file called `Door.tsx` in `react-app-tutorial/src` directory and copy the below snippet into it.

Door.tsx:
```js
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export interface DoorProps {
  door: RomiCore.Door;
  doorState: RomiCore.DoorState;
  onOpenClick?(): void;
  onCloseClick?(): void;
}

export const Door = (props: DoorProps) => {
  const { doorState } = props;
  const modeString = doorState ? doorModeString(doorState.current_mode) : 'Unknown';
  return (
    <span>
      State: {modeString}
      <button>Open</button>
      <button>Close</button>
    </span>
  );
};

function doorModeString(doorMode: RomiCore.DoorMode): string {
  switch (doorMode.value) {
    case RomiCore.DoorMode.MODE_OPEN:
      return 'Open';
    case RomiCore.DoorMode.MODE_CLOSED:
      return 'Closed';
    case RomiCore.DoorMode.MODE_MOVING:
      return 'Moving';
    default:
      return 'Unknown';
  }
}

export default Door;
```

Note here that we are using `@osrf/romi-js-core-interfaces`, this package provides typings information for core messages used by RoMi, if you are using an IDE with typescript autocomplete support, you can easily see the various fields and constants used by the `Door` and `DoorState` messages.

Take the `doorModedString` function for example, using `@osrf/romi-js-core-interfaces` allows us to switch based on the door mode constant (MODE_OPEN, MODE_CLOSED, MODE_MOVING). Normally you would have to refer to the RoMi manual or the ros2 message definition and map the constants manually like this:

```js
function doorModeString(doorMode: RomiCore.DoorMode): string {
  switch (doorMode.value) {
    // values based on ros2 message definitions
    case 2:
      return 'Open';
    case 0:
      return 'Closed';
    case 1:
      return 'Moving';
    default:
      return 'Unknown';
  }
}
```

Let's try running this and see if it works, change `App.tsx` to this

App.tsx:
```js
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import Door from './Door';

function App() {
  const exampleDoor: RomiCore.Door = {
    name: 'example_door',
    door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
    motion_direction: 1,
    motion_range: 1,
    v1_x: 0,
    v1_y: 0,
    v2_x: 0,
    v2_y: 0,
  };
  const exampleDoorState: RomiCore.DoorState = {
    door_name: 'example_door',
    door_time: RomiCore.toRosTime(new Date()),
    current_mode: {
      value: RomiCore.DoorMode.MODE_CLOSED,
    },
  };

  return <Door door={exampleDoor} doorState={exampleDoorState} />;
}

export default App;
```

![Door](ui-resources/door.png)

For now we are hardcoding a fake door, later on we will look at how to we can obtain a list of doors from RoMi.

## Obtaing List of Doors

## Listening for Door States

## Sending Door Requests