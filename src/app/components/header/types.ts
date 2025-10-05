export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface HeaderProps extends BaseProps {
  children?: React.ReactNode;
}
