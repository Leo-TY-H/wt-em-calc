// Pages serves the interface; Oracle runs the calculations.
// Local and Oracle-hosted copies use their own same-origin API.
window.EM_CONFIG={
  apiBase: location.hostname === 'neothunderism.pages.dev'
    ? 'https://150.136.64.13.sslip.io'
    : ''
};
