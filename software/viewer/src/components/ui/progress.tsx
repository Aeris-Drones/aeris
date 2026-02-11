import * as React from "react"
import * as ProgressPrimitive from "@radix-ui/react-progress"
import { cva } from "class-variance-authority"

import { cn } from "@/lib/utils"

/**
 * Style variants for the Progress indicator.
 *
 * Maps semantic status values to theme colors for consistent visual
 * feedback across the GCS (e.g., battery levels, mission progress,
 * file upload status).
 */
const progressVariants = cva("h-full w-full flex-1 bg-primary transition-all", {
    variants: {
        variant: {
            default: "bg-primary",
            success: "bg-success",
            warning: "bg-warning",
            danger: "bg-danger",
        }
    },
    defaultVariants: {
        variant: "default"
    }
})

/**
 * Linear progress indicator with semantic color variants.
 *
 * Extends Radix UI Progress with AERIS-specific status colors.
 * Used for telemetry displays (battery, signal strength), mission
 * progress, and file operation feedback.
 *
 * The indicatorColor prop allows dynamic color changes based on
 * thresholds (e.g., battery < 20% triggers danger variant).
 */
const Progress = React.forwardRef<
  React.ElementRef<typeof ProgressPrimitive.Root>,
  React.ComponentPropsWithoutRef<typeof ProgressPrimitive.Root> & {
      indicatorColor?: "default" | "success" | "warning" | "danger"
  }
>(({ className, value, indicatorColor = "default", ...props }, ref) => (
  <ProgressPrimitive.Root
    ref={ref}
    className={cn(
      "relative h-2 w-full overflow-hidden rounded-full bg-secondary",
      className
    )}
    {...props}
  >
    <ProgressPrimitive.Indicator
      className={progressVariants({ variant: indicatorColor })}
      style={{ transform: `translateX(-${100 - (value || 0)}%)` }}
    />
  </ProgressPrimitive.Root>
))
Progress.displayName = ProgressPrimitive.Root.displayName

export { Progress }
