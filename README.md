# Motion receiver

It's a motion receiver base on the websocket. It use for receiving the command from the client.

## Platforms

Linux

## Install

 - you must konw that the command received will be sended to ttyS1. if you want to handle it directly, you can modify the 'mc/motion_motor_ctrl.c', handle without 'usart'.
 - you can see the data protocol in the 'mc/motion_motor_ctrl.c'.
 - the bind port is 8000 default.

> make

> ./motion_receiver

    send command from the websocket client.

## Reference

[mongoose](https://github.com/cesanta/mongoose)

## [About]
> Me

    Author     : Kevin Wei
    Profession : Embedded software engineer(Linux)
    Industry   : AI, Robot, IOT
    Email      : kkcoding@qq.com

> My Home - [www.xweikk.com](http://www.xweikk.com:8080) (developing)

