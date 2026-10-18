import requests
from openpilot.common.params import Params
from openpilot.system.version import get_version

class PingulinkApi:
  def __init__(self, dongle_id):
    self.dongle_id = dongle_id
    self.params = Params()
    self.session = requests.Session()

  def request(self, method, endpoint, timeout=10, access_token=None, json=None, **params):
    base_url = self.params.get("PingulinkUrl")
    if not base_url:
      return None
      
    base_url = base_url.rstrip('/')

    if access_token is None:
      access_token = self.params.get("PingulinkToken")

    headers = {}
    if access_token:
      # We use Bearer token for Pingulink
      headers['Authorization'] = f"Bearer {access_token}"
    
    headers['User-Agent'] = f"starpingupilot-{get_version()}"
    
    try:
      return self.session.request(method, f"{base_url}/{endpoint}", timeout=timeout, headers=headers, json=json, params=params)
    except Exception:
      return None

  def get(self, endpoint, **kwargs):
    return self.request('GET', endpoint, **kwargs)

  def post(self, endpoint, **kwargs):
    return self.request('POST', endpoint, **kwargs)

  def get_pending_uploads(self):
    return self.get(f"api/uploads/pending/{self.dongle_id}")

  def get_token(self):
    return self.params.get("PingulinkToken")
