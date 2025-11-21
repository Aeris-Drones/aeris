import * as React from "react"
import * as ProgressPrimitive from "@radix-ui/react-progress"
import { cva, type VariantProps } from "class-variance-authority"

import { cn } from "@/lib/utils"

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

// Since standard Radix Progress doesn't support variants easily on the Indicator without passing props down,
// We will extend the component to accept a status variant.

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
