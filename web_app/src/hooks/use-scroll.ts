import React, { useEffect, useState } from "react";

export function useScroll(ref: React.MutableRefObject<HTMLElement>) {
  const [state, setState] = useState<{
    x: number;
    y: number;
  }>({
    x: 0,
    y: 0,
  });

  useEffect(() => {
    const handler = () => {
      if (ref.current) {
        setState({
          x: ref.current.scrollLeft,
          y: ref.current.scrollTop,
        });
      }
    };

    if (ref.current) {
      ref.current.addEventListener("scroll", handler, {
        capture: false,
        passive: true,
      });
    }

    return () => {
      if (ref.current) {
        ref.current.removeEventListener("scroll", handler);
      }
    };
  }, [ref]);

  return state;
}
