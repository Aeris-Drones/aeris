import type { Variants, Transition } from 'framer-motion';

export const transitions = {
  fast: {
    type: 'tween',
    duration: 0.15,
    ease: [0.19, 1, 0.22, 1],
  } as Transition,

  normal: {
    type: 'tween',
    duration: 0.25,
    ease: [0.19, 1, 0.22, 1],
  } as Transition,

  slow: {
    type: 'tween',
    duration: 0.4,
    ease: [0.19, 1, 0.22, 1],
  } as Transition,

  spring: {
    type: 'spring',
    stiffness: 400,
    damping: 30,
  } as Transition,

  springBouncy: {
    type: 'spring',
    stiffness: 300,
    damping: 20,
  } as Transition,

  springGentle: {
    type: 'spring',
    stiffness: 200,
    damping: 25,
  } as Transition,
} as const;

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

export function getStaggerConfig(itemCount: number, baseDelay = 0.05) {
  return {
    staggerChildren: Math.max(0.02, baseDelay / Math.sqrt(itemCount)),
    delayChildren: 0.1,
  };
}

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
