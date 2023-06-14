import React, { useEffect, useRef } from "react";
import { HiOutlineEmojiSad } from "react-icons/hi";
import { Icon } from "@chakra-ui/react";


interface Props {}

const LiveStream: React.FC<Props> = (props) => {
  const [videoAvaiability, setVideoAvaiability] =
    React.useState<boolean>(false);

  const videoFeedRef = useRef() as any;

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8000/feed_image");

    socket.onmessage = function (event) {
      setVideoAvaiability(true);
      const url = URL.createObjectURL(
        new Blob([event.data], { type: "image/jpeg" })
      );
      if (videoFeedRef.current) {
        videoFeedRef.current.src = url;
      }
    };

    socket.onerror = function (error) {
      console.log(`WebSocket Error: ${error}`);
      setVideoAvaiability(false);
    };

    // Limpar a conexão WebSocket ao desmontar o componente
    return () => {
      setVideoAvaiability(false);
      socket.close();
    };
  }, []);

  return (
      
        <div style={{width:"50%",height:"50%", marginLeft:"auto", marginRight:"auto", alignItems:"center", padding:"20px", backgroundColor:"gray", borderRadius:"10px", textAlign:"center" }}> 
        <h2>Imagem da Webcam</h2>
        {videoAvaiability ? (
          <img
            width={"100%"}
            height={"100%"}
            id="videoFeed"
            src=""
            alt="Video Feed"
            ref={videoFeedRef}
          />
          
        ) : (
          <>
            <Icon as={HiOutlineEmojiSad} color={"blue.700"} w={59} h={59} />
            <p className="text-2xl text-blue-gerdau-mid select-none">
              No live stream available
            </p>
          </>
        )}
        </div>
  );
};

export default LiveStream;
