"use client";

import { useEffect } from "react";
import ROSLIB from "roslib";
import type { KinematicsData } from "@/types/KinematicsData";

export function useRosListener(
  onMessage?: (msg: KinematicsData) => void
) {
  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: "ws://localhost:9090", // adjust if your bridge runs elsewhere
    });

    ros.on("connection", () => {
      console.log("Connected to ROS bridge");
    });

    ros.on("error", (error: any) => {
      console.error("Error connecting to ROS bridge:", error);
    });

    ros.on("close", () => {
      console.log("Connection to ROS bridge closed");
    });

    const listener = new ROSLIB.Topic({
      ros,
      name: "/kinematics_data",
      messageType: "moveit_kinematics/msg/KinematicsData",
    });

    listener.subscribe((message: KinematicsData) => {
      if (onMessage) {
        onMessage(message);
      } else {
        console.log("Received KinematicsData:", message);
      }
    });

    return () => {
      listener.unsubscribe();
      ros.close();
    };
  }, [onMessage]);
} 