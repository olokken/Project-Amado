import { ManualControl } from "@/components/control/manualControl";
import { VideoStream } from "@/components/video/videoStream";

export default function Home() {
  return (
    <main className="flex min-h-screen flex-col items-center justify-between p-5">
      <VideoStream />
      <ManualControl />
    </main>
  );
}
