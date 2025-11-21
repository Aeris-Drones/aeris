import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  experimental: {
    turbopack: {
      root: process.cwd(),
    },
  },
};

export default nextConfig;
