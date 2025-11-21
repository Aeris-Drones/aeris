import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // @ts-ignore - turbopack is a valid config but types might be missing
  turbopack: {
    root: process.cwd(),
  },
};

export default nextConfig;
