import * as React from "react"
import { cva, type VariantProps } from "class-variance-authority"
import { cn } from "@/lib/utils"

/**
 * Style variants for the Badge component.
 *
 * Extends the base shadcn/ui variants with AERIS-specific semantic colors
 * (success, warning, danger, info) for consistent status indication across
 * the GCS. All variants maintain WCAG 2.1 AA contrast ratios.
 */
const badgeVariants = cva(
  "inline-flex items-center rounded-md border px-2.5 py-0.5 text-xs font-semibold transition-colors focus:outline-none focus:ring-2 focus:ring-ring focus:ring-offset-2",
  {
    variants: {
      variant: {
        default:
          "border-transparent bg-primary text-primary-foreground shadow hover:bg-primary/80",
        secondary:
          "border-transparent bg-secondary text-secondary-foreground hover:bg-secondary/80",
        destructive:
          "border-transparent bg-destructive text-destructive-foreground shadow hover:bg-destructive/80",
        outline: "text-foreground",
        success: "border-transparent bg-success text-white shadow hover:bg-success/80",
        warning: "border-transparent bg-warning text-white shadow hover:bg-warning/80",
        danger: "border-transparent bg-danger text-white shadow hover:bg-danger/80",
        info: "border-transparent bg-info text-white shadow hover:bg-info/80",
      },
    },
    defaultVariants: {
      variant: "default",
    },
  }
)

/**
 * Props for the Badge component.
 *
 * Supports all standard div attributes plus variant styling. The component
 * renders as a non-interactive status indicator; use Button for clickable
 * pill-shaped elements.
 */
export interface BadgeProps
  extends React.HTMLAttributes<HTMLDivElement>,
    VariantProps<typeof badgeVariants> {}

/**
 * Status indicator component for labels, counts, and states.
 *
 * Used throughout the GCS for mission status, detection categories, and
 * telemetry indicators. Follows the shadcn/ui pattern for consistency with
 * the broader ecosystem while adding AERIS-specific semantic variants.
 */
function Badge({ className, variant, ...props }: BadgeProps) {
  return (
    <div className={cn(badgeVariants({ variant }), className)} {...props} />
  )
}

export { Badge, badgeVariants }
