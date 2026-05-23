import Link from 'next/link';
import { Crosshair, Wrench } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { cn } from '@/lib/utils';

interface ViewerRouteNavProps {
  currentRoute: 'operations' | 'maintenance';
}

export function ViewerRouteNav({ currentRoute }: ViewerRouteNavProps) {
  return (
    <div className="flex items-center gap-1 rounded-full border border-white/12 bg-black/60 p-1 shadow-[0_0_24px_rgba(0,0,0,0.28)] backdrop-blur-xl">
      <Button
        asChild
        variant="ghost"
        size="sm"
        className={cn(
          'h-9 rounded-full px-3 text-white/75 hover:bg-white/8 hover:text-white',
          currentRoute === 'operations' && 'bg-white/12 text-white'
        )}
      >
        <Link href="/">
          <Crosshair className="mr-2 h-4 w-4" />
          Ops
        </Link>
      </Button>
      <Button
        asChild
        variant="ghost"
        size="sm"
        className={cn(
          'h-9 rounded-full px-3 text-white/75 hover:bg-white/8 hover:text-white',
          currentRoute === 'maintenance' && 'bg-white/12 text-white'
        )}
      >
        <Link href="/maintenance">
          <Wrench className="mr-2 h-4 w-4" />
          Maintenance
        </Link>
      </Button>
    </div>
  );
}
