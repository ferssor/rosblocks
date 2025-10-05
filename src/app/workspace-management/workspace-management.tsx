import { memo } from 'react';
import { WorkspaceManagementProps } from './types';
import useWorkspaceManagementHook from './workspace-management.hook';
import styles from './workspace-management.styles.module.css';

function WorkspaceManagement(props: WorkspaceManagementProps) {
  const { className = '', style, children } = props;
  const { t, state, handlers } = useWorkspaceManagementHook();

  return (
    <div
      id={props.id}
      className={(styles.container + ' ' + (className || '')).trim()}
      style={style}
      onClick={handlers.onClick}
    >
      <h2 className={styles.title}>{t('title')}</h2>
      {state.showContent && children}
    </div>
  );
}

export default memo(WorkspaceManagement);
