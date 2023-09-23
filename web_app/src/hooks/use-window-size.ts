import { useEffect, useState } from "react";

export function useWindowSize() {
  const [size, setSize] = useState<{
    width: number;
    height: number;
  }>({
    width: window.innerWidth,
    height: window.innerHeight,
  });

  useEffect(() => {
    let timer:NodeJS.Timeout;

    const resizeWindow = () => {
      if (timer) {
        clearTimeout(timer);
      }
      timer = setTimeout(function () {
        setSize({
          width: window.innerWidth,
          height: window.innerHeight,
        });
      }, 200);
    };
    window.addEventListener("resize", resizeWindow);
    return () => {
      window.removeEventListener("resize", resizeWindow);
    };
  }, []);

  return size;
}
