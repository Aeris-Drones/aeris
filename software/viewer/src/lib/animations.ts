import type { Variants, Transition } from 'framer-motion';

/**
 * Animation Configuration for Framer Motion
 *
 * This module provides reusable transition presets and variant definitions
 * for consistent motion design across the application.
 *
 * Performance Considerations:
 * - Prefer 'tween' over 'spring' for simple opacity/transform animations
 * - Use transform and opacity only (GPU-accelerated properties)
 * - Stagger delays are calculated based on list size to maintain perceived speed
 */

/**
 * Base transition presets for consistent timing.
 *
 * Easing function [0.19, 1, 0.22, 1] is an outExpo curve that starts fast
 * and decelerates smoothly, providing a responsive yet polished feel.
 */
export const transitions = {
  /** Quick feedback (150ms), use for hover states and micro-interactions */
  fast: {
    type: 'tween',
    duration: 0.15,
    ease: [0.19, 1, 0.22, 1],
  } as Transition,

  /** Standard transition (250ms), use for most UI state changes */
  normal: {
    type: 'tween',
    duration: 0.25,
    ease: [0.19, 1, 0.22, 1],
  } as Transition,

  /** Emphasis transition (400ms), use for prominent elements */
  slow: {
    type: 'tween',
    duration: 0.4,
    ease: [0.19, 1, 0.22, 1],
  } as Transition,

  /** Snappy spring for responsive interactions */
  spring: {
    type: 'spring',
    stiffness: 400,
    damping: 30,
  } as Transition,

  /** Bouncy spring for playful/celebratory animations */
  springBouncy: {
    type: 'spring',
    stiffness: 300,
    damping: 20,
  } as Transition,

  /** Gentle spring for subtle, refined motion */
  springGentle: {
    type: 'spring',
    stiffness: 200,
    damping: 25,
  } as Transition,
} as const;

/**
 * Panel entrance/exit variants with scale and fade.
 *
 * Use for modals, popovers, and floating panels.
 */
export const panelVariants: Variants = {
  hidden: {
    opacity: 0,
    scale: 0.95,
    y: 10,
  },
  visible: {
    opacity: 1,
    scale: 1,
    y: 0,
    transition: transitions.spring,
  },
  exit: {
    opacity: 0,
    scale: 0.95,
    y: 10,
    transition: transitions.fast,
  },
};

/**
 * Card variants with hover and tap interactions.
 *
 * Includes scale feedback on hover (1.02x) and tap (0.98x).
 * Use for clickable cards and list items.
 */
export const cardVariants: Variants = {
  hidden: {
    opacity: 0,
    y: 20,
    scale: 0.95,
  },
  visible: {
    opacity: 1,
    y: 0,
    scale: 1,
    transition: transitions.spring,
  },
  exit: {
    opacity: 0,
    y: -10,
    scale: 0.95,
    transition: transitions.fast,
  },
  hover: {
    scale: 1.02,
    transition: transitions.fast,
  },
  tap: {
    scale: 0.98,
    transition: { duration: 0.1 },
  },
};

/**
 * List container variants with staggered children.
 *
 * staggerChildren: 50ms delay between each child animation
 * delayChildren: 100ms initial delay before first child
 */
export const listContainerVariants: Variants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.05,
      delayChildren: 0.1,
    },
  },
  exit: {
    opacity: 0,
    transition: {
      staggerChildren: 0.03,
      staggerDirection: -1,
    },
  },
};

/**
 * Individual list item variants for use within listContainerVariants.
 *
 * Slides in from left (-20px) and exits to right (+20px).
 */
export const listItemVariants: Variants = {
  hidden: {
    opacity: 0,
    x: -20,
  },
  visible: {
    opacity: 1,
    x: 0,
    transition: transitions.spring,
  },
  exit: {
    opacity: 0,
    x: 20,
    transition: transitions.fast,
  },
};

/** Simple fade in/out without transform */
export const fadeVariants: Variants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: transitions.normal,
  },
  exit: {
    opacity: 0,
    transition: transitions.fast,
  },
};

/** Scale up entrance with bouncy spring, scale down exit */
export const scaleVariants: Variants = {
  hidden: {
    opacity: 0,
    scale: 0.8,
  },
  visible: {
    opacity: 1,
    scale: 1,
    transition: transitions.springBouncy,
  },
  exit: {
    opacity: 0,
    scale: 0.8,
    transition: transitions.fast,
  },
};

/** Slide up from bottom of viewport (100% -> 0%) */
export const slideUpVariants: Variants = {
  hidden: {
    y: '100%',
  },
  visible: {
    y: 0,
    transition: transitions.spring,
  },
  exit: {
    y: '100%',
    transition: transitions.normal,
  },
};

/** Slide down from top of viewport (-100% -> 0%) */
export const slideDownVariants: Variants = {
  hidden: {
    y: '-100%',
  },
  visible: {
    y: 0,
    transition: transitions.spring,
  },
  exit: {
    y: '-100%',
    transition: transitions.normal,
  },
};

/** Slide in from right edge (100% -> 0%) */
export const slideLeftVariants: Variants = {
  hidden: {
    x: '100%',
  },
  visible: {
    x: 0,
    transition: transitions.spring,
  },
  exit: {
    x: '100%',
    transition: transitions.normal,
  },
};

/** Slide in from left edge (-100% -> 0%) */
export const slideRightVariants: Variants = {
  hidden: {
    x: '-100%',
  },
  visible: {
    x: 0,
    transition: transitions.spring,
  },
  exit: {
    x: '-100%',
    transition: transitions.normal,
  },
};

/**
 * Pulsing ring animation for status indicators.
 *
 * Creates expanding ring effect using box-shadow animation.
 * Requires CSS variable --confidence-high to be defined.
 */
export const pulseVariants: Variants = {
  initial: {
    boxShadow: '0 0 0 0 var(--confidence-high)',
  },
  pulse: {
    boxShadow: [
      '0 0 0 0 var(--confidence-high)',
      '0 0 0 8px transparent',
      '0 0 0 0 var(--confidence-high)',
    ],
    transition: {
      duration: 1.5,
      repeat: Infinity,
      ease: 'easeInOut',
    },
  },
};

/**
 * Status glow variants with increasing urgency.
 *
 * - nominal: Slow 2s pulse (healthy state)
 * - caution: Medium 1.5s pulse (warning state)
 * - critical: Fast 0.8s pulse (alert state)
 */
export const glowVariants: Variants = {
  nominal: {
    boxShadow: '0 0 10px var(--success)',
    transition: {
      duration: 2,
      repeat: Infinity,
      repeatType: 'reverse',
      ease: 'easeInOut',
    },
  },
  caution: {
    boxShadow: [
      '0 0 10px var(--warning)',
      '0 0 20px var(--warning)',
      '0 0 10px var(--warning)',
    ],
    transition: {
      duration: 1.5,
      repeat: Infinity,
      ease: 'easeInOut',
    },
  },
  critical: {
    boxShadow: [
      '0 0 10px var(--danger)',
      '0 0 25px var(--danger)',
      '0 0 10px var(--danger)',
    ],
    transition: {
      duration: 0.8,
      repeat: Infinity,
      ease: 'easeInOut',
    },
  },
};

/** Number/count animation with vertical slide */
export const numberVariants: Variants = {
  initial: { opacity: 0, y: 10 },
  animate: {
    opacity: 1,
    y: 0,
    transition: transitions.spring,
  },
  exit: {
    opacity: 0,
    y: -10,
    transition: transitions.fast,
  },
};

/**
 * SVG progress ring animation.
 *
 * @param progress - Value from 0 to 1 representing completion
 * Uses spring physics for pathLength animation.
 */
export const progressRingVariants: Variants = {
  hidden: {
    pathLength: 0,
    opacity: 0,
  },
  visible: (progress: number) => ({
    pathLength: progress,
    opacity: 1,
    transition: {
      pathLength: {
        type: 'spring',
        stiffness: 50,
        damping: 20,
      },
      opacity: transitions.fast,
    },
  }),
};

/** Tooltip entrance/exit with subtle scale */
export const tooltipVariants: Variants = {
  hidden: {
    opacity: 0,
    scale: 0.9,
    y: 4,
  },
  visible: {
    opacity: 1,
    scale: 1,
    y: 0,
    transition: {
      type: 'spring',
      stiffness: 500,
      damping: 30,
    },
  },
  exit: {
    opacity: 0,
    scale: 0.9,
    y: 4,
    transition: { duration: 0.1 },
  },
};

/**
 * Skeleton loading shimmer effect.
 *
 * Animates background-position to create horizontal shimmer.
 * Requires a gradient background with at least 200% width.
 */
export const skeletonVariants: Variants = {
  initial: {
    backgroundPosition: '-200% 0',
  },
  animate: {
    backgroundPosition: '200% 0',
    transition: {
      duration: 1.5,
      repeat: Infinity,
      ease: 'linear',
    },
  },
};

/** Button press feedback with hover and tap states */
export const buttonVariants: Variants = {
  idle: { scale: 1 },
  hover: {
    scale: 1.02,
    transition: transitions.fast,
  },
  tap: {
    scale: 0.95,
    transition: { duration: 0.1 },
  },
};

/**
 * Detection card styling by sensor type.
 *
 * Maps sensor types to their theme colors and glow effects.
 */
export const detectionCardVariants = {
  thermal: {
    borderColor: 'var(--sensor-thermal)',
    boxShadow: '0 0 15px var(--sensor-thermal-glow)',
  },
  acoustic: {
    borderColor: 'var(--sensor-acoustic)',
    boxShadow: '0 0 15px var(--sensor-acoustic-glow)',
  },
  gas: {
    borderColor: 'var(--sensor-gas)',
    boxShadow: '0 0 15px var(--sensor-gas-glow)',
  },
} as const;

/**
 * Calculates stagger configuration based on list size.
 *
 * As list size increases, individual stagger delay decreases to maintain
 * total animation duration. Formula: baseDelay / sqrt(itemCount)
 *
 * @param itemCount - Number of items in the list
 * @param baseDelay - Base delay in seconds (default: 0.05)
 * @returns Stagger configuration object
 */
export function getStaggerConfig(itemCount: number, baseDelay = 0.05) {
  return {
    staggerChildren: Math.max(0.02, baseDelay / Math.sqrt(itemCount)),
    delayChildren: 0.1,
  };
}

/**
 * Preset collection for common animation patterns.
 *
 * Usage: import { presets } from './animations'
 *        <motion.div variants={presets.card} ... />
 */
export const presets = {
  panel: panelVariants,
  card: cardVariants,
  list: listContainerVariants,
  listItem: listItemVariants,
  fade: fadeVariants,
  scale: scaleVariants,
  slideUp: slideUpVariants,
  slideDown: slideDownVariants,
  slideLeft: slideLeftVariants,
  slideRight: slideRightVariants,
  pulse: pulseVariants,
  glow: glowVariants,
  number: numberVariants,
  progressRing: progressRingVariants,
  tooltip: tooltipVariants,
  skeleton: skeletonVariants,
  button: buttonVariants,
} as const;
