import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Aeris GCS",
  description: "Ground Control Station for Aeris Swarm",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="dark">
      <body className="antialiased bg-background text-foreground">
        {children}
      </body>
    </html>
  );
}
