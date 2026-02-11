import React, { CSSProperties } from "react";

import { cn } from "@/lib/utils";

/**
 * Props for the ShimmerButton component.
 *
 * Extends standard button attributes with customization options for the
 * animated shimmer effect. All styling props use CSS custom properties
 * for performant GPU-accelerated animations.
 */
export interface ShimmerButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  /** Color of the shimmer animation (CSS color value). Default: #ffffff */
  shimmerColor?: string;
  /** Thickness of the shimmer border. Default: 0.05em */
  shimmerSize?: string;
  /** Border radius of the button. Default: 100px */
  borderRadius?: string;
  /** Duration of the shimmer animation. Default: 3s */
  shimmerDuration?: string;
  /** Background color behind the shimmer. Default: rgba(0, 0, 0, 1) */
  background?: string;
  className?: string;
  children?: React.ReactNode;
}

/**
 * Button with an animated shimmer border effect.
 *
 * Uses CSS custom properties and conic gradients for a performant,
 * GPU-accelerated animation. The shimmer rotates around the button border
 * to draw attention to primary actions.
 *
 * Commonly used for high-priority CTAs like emergency controls or
 * critical mission actions that require operator attention.
 */
const ShimmerButton = React.forwardRef<HTMLButtonElement, ShimmerButtonProps>(
  (
    {
      shimmerColor = "#ffffff",
      shimmerSize = "0.05em",
      shimmerDuration = "3s",
      borderRadius = "100px",
      background = "rgba(0, 0, 0, 1)",
      className,
      children,
      ...props
    },
    ref,
  ) => {
    return (
      <button
        style={
          {
            "--spread": "90deg",
            "--shimmer-color": shimmerColor,
            "--radius": borderRadius,
            "--speed": shimmerDuration,
            "--cut": shimmerSize,
            "--bg": background,
          } as CSSProperties
        }
        className={cn(
          "group relative z-0 flex cursor-pointer items-center justify-center overflow-hidden whitespace-nowrap border border-white/10 px-6 py-3 text-white [background:var(--bg)] [border-radius:var(--radius)] dark:text-black",
          "transform-gpu transition-transform duration-300 ease-in-out active:translate-y-px",
          className,
        )}
        ref={ref}
        {...props}
      >
        <div
          className={cn(
            "-z-30 blur-[2px]",
            "absolute inset-0 overflow-visible [container-type:size]",
          )}
        >
          <div className="absolute inset-0 h-[100cqh] animate-shimmer-slide [aspect-ratio:1] [border-radius:0] [mask:none]">
            <div className="animate-spin-around absolute -inset-full w-auto rotate-0 [background:conic-gradient(from_calc(270deg-(var(--spread)*0.5)),transparent_0,var(--shimmer-color)_var(--spread),transparent_var(--spread))] [translate:0_0]" />
          </div>
        </div>
        {children}

        <div
          className={cn(
            "insert-0 absolute size-full",

            "rounded-2xl px-4 py-1.5 text-sm font-medium shadow-[inset_0_-8px_10px_#ffffff1f]",

            "transform-gpu transition-all duration-300 ease-in-out",

            "group-hover:shadow-[inset_0_-6px_10px_#ffffff3f]",

            "group-active:shadow-[inset_0_-10px_10px_#ffffff3f]",
          )}
        />

        <div
          className={cn(
            "absolute -z-20 [background:var(--bg)] [border-radius:var(--radius)] [inset:var(--cut)]",
          )}
        />
      </button>
    );
  },
);

ShimmerButton.displayName = "ShimmerButton";

export { ShimmerButton };
