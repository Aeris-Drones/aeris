import type { Metadata } from "next";
import "./globals.css";

/**
 * Root layout for the Aeris Ground Control Station.
 *
 * The GCS is a single-page application with a fixed dark theme
 * to reduce eye strain during extended field operations and to
 * provide consistent contrast for map visualization components.
 */
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
