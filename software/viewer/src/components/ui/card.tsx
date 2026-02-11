import * as React from "react"
import { cn } from "@/lib/utils"

/**
 * Container component for grouping related content.
 *
 * Implements the glassmorphism design system with backdrop blur and
 * semi-transparent backgrounds. Used throughout the GCS for panels,
 * sheets, and modal content to maintain visual consistency.
 */
const Card = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    className={cn(
      "rounded-xl border bg-card text-card-foreground shadow-sm",
      // Glassmorphism effect
      "bg-opacity-60 backdrop-blur-xl border-white/10",
      className
    )}
    {...props}
  />
))
Card.displayName = "Card"

/**
 * Header section for Card components.
 *
 * Provides consistent spacing and layout for titles and actions
 * at the top of card containers.
 */
const CardHeader = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    className={cn("flex flex-col space-y-1.5 p-6", className)}
    {...props}
  />
))
CardHeader.displayName = "CardHeader"

/**
 * Title element for Card components.
 *
 * Uses semibold weight and tight tracking for hierarchy within
 * the glassmorphism UI. Renders as a div for flexibility.
 */
const CardTitle = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    className={cn("font-semibold leading-none tracking-tight", className)}
    {...props}
  />
))
CardTitle.displayName = "CardTitle"

/**
 * Secondary text element for Card components.
 *
 * Uses muted foreground color for de-emphasized descriptive content
 * beneath the card title.
 */
const CardDescription = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    className={cn("text-sm text-muted-foreground", className)}
    {...props}
  />
))
CardDescription.displayName = "CardDescription"

/**
 * Primary content area for Card components.
 *
 * Provides consistent padding and removes top padding when following
 * a CardHeader to avoid double-spacing.
 */
const CardContent = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div ref={ref} className={cn("p-6 pt-0", className)} {...props} />
))
CardContent.displayName = "CardContent"

/**
 * Footer section for Card components.
 *
 * Typically used for action buttons or secondary information.
 * Aligns items horizontally with consistent spacing.
 */
const CardFooter = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    className={cn("flex items-center p-6 pt-0", className)}
    {...props}
  />
))
CardFooter.displayName = "CardFooter"

export { Card, CardHeader, CardFooter, CardTitle, CardDescription, CardContent }
