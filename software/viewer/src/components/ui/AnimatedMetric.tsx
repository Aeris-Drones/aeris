'use client';

import React, { useEffect, useRef } from 'react';
import { motion, useSpring, useTransform, animate } from 'framer-motion';
import { cn } from '@/lib/utils';
import { durations } from '@/lib/design-tokens';
import { ArrowUp, ArrowDown, Minus } from 'lucide-react';

type StatusLevel = 'nominal' | 'caution' | 'warning' | 'critical';
type TrendDirection = 'up' | 'down' | 'stable';

interface AnimatedMetricProps {
  value: number;
  previousValue?: number;
  unit?: string;
  label?: string;
  status?: StatusLevel;
  precision?: number;
  showTrend?: boolean;
  size?: 'sm' | 'md' | 'lg' | 'xl';
  className?: string;
  valueClassName?: string;
  labelClassName?: string;
}

const statusColors: Record<StatusLevel, string> = {
  nominal: 'text-success',
  caution: 'text-warning',
  warning: 'text-warning',
  critical: 'text-danger',
};

const statusGlows: Record<StatusLevel, string> = {
  nominal: '',
  caution: 'drop-shadow-[0_0_8px_var(--warning)]',
  warning: 'drop-shadow-[0_0_8px_var(--warning)]',
  critical: 'drop-shadow-[0_0_10px_var(--danger)] animate-pulse',
};

const sizeClasses = {
  sm: {
    value: 'text-lg font-semibold',
    unit: 'text-xs',
    label: 'text-xs',
    trend: 'w-3 h-3',
  },
  md: {
    value: 'text-2xl font-semibold',
    unit: 'text-sm',
    label: 'text-xs',
    trend: 'w-4 h-4',
  },
  lg: {
    value: 'text-3xl font-bold',
    unit: 'text-base',
    label: 'text-sm',
    trend: 'w-5 h-5',
  },
  xl: {
    value: 'text-5xl font-bold',
    unit: 'text-lg',
    label: 'text-base',
    trend: 'w-6 h-6',
  },
};

function getTrend(current: number, previous?: number): TrendDirection {
  if (previous === undefined) return 'stable';
  const diff = current - previous;
  const threshold = Math.abs(previous) * 0.01; // 1% change threshold
  if (diff > threshold) return 'up';
  if (diff < -threshold) return 'down';
  return 'stable';
}

const TrendIcon: React.FC<{ trend: TrendDirection; className?: string }> = ({
  trend,
  className,
}) => {
  switch (trend) {
    case 'up':
      return <ArrowUp className={cn(className, 'text-success')} />;
    case 'down':
      return <ArrowDown className={cn(className, 'text-danger')} />;
    default:
      return <Minus className={cn(className, 'text-muted-foreground')} />;
  }
};

// Animated number component using spring physics
function AnimatedNumber({
  value,
  precision = 0,
  className,
}: {
  value: number;
  precision?: number;
  className?: string;
}) {
  const ref = useRef<HTMLSpanElement>(null);
  const prevValue = useRef(value);

  useEffect(() => {
    const node = ref.current;
    if (!node) return;

    const controls = animate(prevValue.current, value, {
      duration: durations.normal / 1000,
      ease: [0.19, 1, 0.22, 1],
      onUpdate(val) {
        node.textContent = val.toFixed(precision);
      },
    });

    prevValue.current = value;

    return () => controls.stop();
  }, [value, precision]);

  return (
    <span ref={ref} className={cn('tabular-nums font-mono', className)}>
      {value.toFixed(precision)}
    </span>
  );
}

export function AnimatedMetric({
  value,
  previousValue,
  unit,
  label,
  status = 'nominal',
  precision = 1,
  showTrend = false,
  size = 'md',
  className,
  valueClassName,
  labelClassName,
}: AnimatedMetricProps) {
  const trend = getTrend(value, previousValue);
  const sizes = sizeClasses[size];

  return (
    <div className={cn('flex flex-col', className)}>
      {label && (
        <span
          className={cn(
            sizes.label,
            'text-muted-foreground uppercase tracking-wider mb-1',
            labelClassName
          )}
        >
          {label}
        </span>
      )}

      <div className="flex items-baseline gap-1">
        <motion.span
          className={cn(
            sizes.value,
            statusColors[status],
            statusGlows[status],
            valueClassName
          )}
          initial={{ opacity: 0, y: 5 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.2 }}
        >
          <AnimatedNumber value={value} precision={precision} />
        </motion.span>

        {unit && (
          <span
            className={cn(sizes.unit, 'text-muted-foreground font-medium')}
          >
            {unit}
          </span>
        )}

        {showTrend && (
          <motion.div
            initial={{ opacity: 0, scale: 0.5 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ delay: 0.1 }}
          >
            <TrendIcon trend={trend} className={sizes.trend} />
          </motion.div>
        )}
      </div>
    </div>
  );
}

// Compact inline variant
interface InlineMetricProps {
  value: number;
  unit?: string;
  precision?: number;
  status?: StatusLevel;
  className?: string;
}

export function InlineMetric({
  value,
  unit,
  precision = 0,
  status = 'nominal',
  className,
}: InlineMetricProps) {
  return (
    <span
      className={cn(
        'inline-flex items-baseline gap-0.5 font-mono tabular-nums',
        statusColors[status],
        className
      )}
    >
      <AnimatedNumber value={value} precision={precision} />
      {unit && (
        <span className="text-[0.75em] text-muted-foreground">{unit}</span>
      )}
    </span>
  );
}

// Large hero metric for KPI displays
interface HeroMetricProps extends AnimatedMetricProps {
  description?: string;
}

export function HeroMetric({
  value,
  unit,
  label,
  description,
  status = 'nominal',
  precision = 0,
  className,
}: HeroMetricProps) {
  return (
    <div className={cn('text-center', className)}>
      <AnimatedMetric
        value={value}
        unit={unit}
        label={label}
        status={status}
        precision={precision}
        size="xl"
        className="items-center"
      />
      {description && (
        <p className="text-sm text-muted-foreground mt-2">{description}</p>
      )}
    </div>
  );
}
