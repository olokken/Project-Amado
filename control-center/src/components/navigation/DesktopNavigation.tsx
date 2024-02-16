import Link from "next/link";
import LogoLink from "../LogoLink";
import navigation from "@/lib/navigation";
import { useRouter } from "next/router";
import {
  NavigationMenu,
  NavigationMenuItem,
  NavigationMenuList,
} from "../ui/NavigationMenu";
import { Button } from "../button";
import { cn } from "@/utils/cn";
import ProfileAvatar from "../ProfileAvatar";

const DesktopNavigation = () => {
  const router = useRouter();

  // TODO: Replace with real auth check
  const isSignedIn = true;

  return (
    <NavigationMenu className="hidden md:flex">
      <div className="flex items-center justify-start">
        <LogoLink />
      </div>

      {isSignedIn && (
        <NavigationMenuList className="gap-x-2">
          {/* Right part of navigation, with the rest */}
          {navigation.map((item, i) => {
            const active = router.pathname === item.href;
            const isTime = item.href === "/timer";

            return (
              <NavigationMenuItem key={i}>
                <Link href={item.href}>
                  <Button
                    variant={active ? "subtle" : "link"}
                    className={cn(
                      isTime && "bg-red-500 text-slate-50 hover:bg-red-600"
                    )}
                  >
                    {item.label}
                  </Button>
                </Link>
              </NavigationMenuItem>
            );
          })}
          <div className="flex items-center justify-center">
            {isSignedIn && (
              <div className="flex items-center justify-center">
                <NavigationMenuItem className="mr-6">
                  <ProfileAvatar />
                </NavigationMenuItem>
              </div>
            )}
          </div>
        </NavigationMenuList>
      )}
    </NavigationMenu>
  );
};
export default DesktopNavigation;
