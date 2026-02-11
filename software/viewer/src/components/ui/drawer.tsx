"use client"

import * as React from "react"
import { Drawer as DrawerPrimitive } from "vaul"

import { cn } from "@/lib/utils"

/**
 * Root drawer component providing slide-out panel functionality.
 *
 * Built on Vaul for smooth mobile-style bottom sheet interactions.
 * Used in the GCS for secondary workflows that don't require full
 * modal attention (e.g., configuration panels, detail views).
 *
 * @see https://github.com/emilkowalski/vaul
 */
const Drawer = ({
  shouldScaleBackground = true,
  ...props
}: React.ComponentProps<typeof DrawerPrimitive.Root>) => (
  <DrawerPrimitive.Root
    shouldScaleBackground={shouldScaleBackground}
    {...props}
  />
)
Drawer.displayName = "Drawer"

/**
 * Trigger element that opens the drawer when clicked.
 * Renders as a button by default; use asChild to customize.
 */
const DrawerTrigger = DrawerPrimitive.Trigger

/**
 * Portal container for rendering drawer content outside the DOM hierarchy.
 * Required for proper z-index stacking and backdrop behavior.
 */
const DrawerPortal = DrawerPrimitive.Portal

/**
 * Close button element for programmatic or explicit drawer dismissal.
 */
const DrawerClose = DrawerPrimitive.Close

/**
 * Backdrop overlay behind the drawer content.
 *
 * Provides visual focus and click-to-dismiss functionality.
 * Fixed positioning ensures coverage regardless of scroll position.
 */
const DrawerOverlay = React.forwardRef<
  React.ElementRef<typeof DrawerPrimitive.Overlay>,
  React.ComponentPropsWithoutRef<typeof DrawerPrimitive.Overlay>
>(({ className, ...props }, ref) => (
  <DrawerPrimitive.Overlay
    ref={ref}
    className={cn("fixed inset-0 z-50 bg-black/80", className)}
    {...props}
  />
))
DrawerOverlay.displayName = DrawerPrimitive.Overlay.displayName

/**
 * Main drawer content container with drag-to-dismiss gesture support.
 *
 * Renders as a fixed bottom panel on mobile, can be customized for
 * desktop via className. Includes a visual drag handle indicator.
 */
const DrawerContent = React.forwardRef<
  React.ElementRef<typeof DrawerPrimitive.Content>,
  React.ComponentPropsWithoutRef<typeof DrawerPrimitive.Content>
>(({ className, children, ...props }, ref) => (
  <DrawerPortal>
    <DrawerOverlay />
    <DrawerPrimitive.Content
      ref={ref}
      className={cn(
        "fixed inset-x-0 bottom-0 z-50 mt-24 flex h-auto flex-col rounded-t-[10px] border bg-background",
        className,
      )}
      {...props}
    >
      <div className="mx-auto mt-4 h-2 w-[100px] rounded-full bg-muted" />
      {children}
    </DrawerPrimitive.Content>
  </DrawerPortal>
))
DrawerContent.displayName = "DrawerContent"

/**
 * Header section for drawer content with consistent styling.
 *
 * Centers text on mobile, left-aligns on larger screens per sm breakpoint.
 */
const DrawerHeader = ({
  className,
  ...props
}: React.HTMLAttributes<HTMLDivElement>) => (
  <div
    className={cn("grid gap-1.5 p-4 text-center sm:text-left", className)}
    {...props}
  />
)
DrawerHeader.displayName = "DrawerHeader"

/**
 * Footer section for drawer actions, typically containing buttons.
 *
 * Uses mt-auto to push content to the bottom of the drawer.
 */
const DrawerFooter = ({
  className,
  ...props
}: React.HTMLAttributes<HTMLDivElement>) => (
  <div
    className={cn("mt-auto flex flex-col gap-2 p-4", className)}
    {...props}
  />
)
DrawerFooter.displayName = "DrawerFooter"

/**
 * Accessible title element for the drawer.
 *
 * Required for screen reader accessibility. Announced when drawer opens.
 */
const DrawerTitle = React.forwardRef<
  React.ElementRef<typeof DrawerPrimitive.Title>,
  React.ComponentPropsWithoutRef<typeof DrawerPrimitive.Title>
>(({ className, ...props }, ref) => (
  <DrawerPrimitive.Title
    ref={ref}
    className={cn(
      "text-lg font-semibold leading-none tracking-tight",
      className,
    )}
    {...props}
  />
))
DrawerTitle.displayName = DrawerPrimitive.Title.displayName

/**
 * Accessible description element for the drawer.
 *
 * Provides additional context for screen reader users.
 * Recommended when the drawer contains complex interactions.
 */
const DrawerDescription = React.forwardRef<
  React.ElementRef<typeof DrawerPrimitive.Description>,
  React.ComponentPropsWithoutRef<typeof DrawerPrimitive.Description>
>(({ className, ...props }, ref) => (
  <DrawerPrimitive.Description
    ref={ref}
    className={cn("text-sm text-muted-foreground", className)}
    {...props}
  />
))
DrawerDescription.displayName = DrawerPrimitive.Description.displayName

export {
  Drawer,
  DrawerPortal,
  DrawerOverlay,
  DrawerTrigger,
  DrawerClose,
  DrawerContent,
  DrawerHeader,
  DrawerFooter,
  DrawerTitle,
  DrawerDescription,
}
