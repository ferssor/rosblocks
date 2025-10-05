import { useTranslation } from 'react-i18next';
import { useState, useCallback } from 'react';
import './i18n';

function useWorkspaceManagementHook() {
  const { t } = useTranslation('workspace_management');
  const [showContent, setShowContent] = useState(true);
  const handleClick = useCallback(function () { setShowContent(function (p) { return !p; }); }, []);
  return { t: t, state: { showContent: showContent }, handlers: { onClick: handleClick } };
}

export default useWorkspaceManagementHook;
