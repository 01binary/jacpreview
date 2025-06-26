"use client";

import { useRosListener } from "@/hooks/useRosListener";

export default function Home() {
  useRosListener();
  return (
    <main>
      <h1>JacPreview Web</h1>
      {/* Add your UI here */}
    </main>
  );
}
