"use client";
import React, { useRef, useEffect } from "react";

export const VideoStream = () => {
  const videoRef = useRef(null);

  useEffect(() => {
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
      navigator.mediaDevices
        .getUserMedia({ video: true })
        .then((stream) => {
          if (videoRef.current) {
            videoRef.current.srcObject = stream;
          }
        })
        .catch((err) => {
          console.log(err);
        });
    }
  }, []);

  return <video ref={videoRef} autoPlay />;
};
