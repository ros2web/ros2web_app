import Button from "@mui/material/Button";
import ButtonGroup, { ButtonGroupProps } from "@mui/material/ButtonGroup";
import WidgetBox from "@/components/WidgetBox";
import { AppEvent } from "@/containers/App/app-state";

type Props = {
  label?: string;
  labels?: string[];
  color?:
    | "inherit"
    | "primary"
    | "secondary"
    | "error"
    | "info"
    | "success"
    | "warning";
  size?: "small" | "medium" | "large";
  variant?: "text" | "outlined" | "contained";
  orientation?: "vertical" | "horizontal";
  disable_elevation?: boolean;
  on_click?: (event: AppEvent) => void;
  justify_content?: string;
  align_items?: string;
};

type ClickEvent = {
  index: number;
  label: string;
};

export default function ButtonWidget(props: Props) {
  const {
    on_click,
    label,
    labels,
    color,
    size,
    variant,
    disable_elevation,
    orientation,
    justify_content,
    align_items,
  } = props;

  const clickHandler = async (value: ClickEvent) => {
    if (on_click) {
      const event: AppEvent = {
        type: "click",
        value,
      };
      on_click(event);
    }
  };
  const buttonLabels = labels || [label || "Button"];

  const buttonGroupProps: ButtonGroupProps = {
    ...(color ? { color } : {}),
    ...(size ? { size } : {}),
    ...(variant ? { variant } : {}),
    ...(orientation ? { orientation } : {}),
    ...(disable_elevation ? { disableElevation: true } : {}),
  };

  return (
    <WidgetBox justify_content={justify_content} align_items={align_items}>
      <ButtonGroup {...buttonGroupProps}>
        {buttonLabels.map((buttonLabel, index) => (
          <Button
            key={index}
            onClick={() =>
              clickHandler({
                index,
                label: buttonLabel,
              })
            }
          >
            {buttonLabel}
          </Button>
        ))}
      </ButtonGroup>
    </WidgetBox>
  );
}
