# Asana_Imports/asana_api_client.py
import os
import requests
import logging
from .asana_error_handler import handle_api_error

BASE_URL = "https://app.asana.com/api/1.0"

class AsanaClient:
    def __init__(self, token, workspace_id):
        self.token = token
        self.workspace_id = workspace_id
        self.base_url = BASE_URL

    def _make_request(self, method, endpoint, params=None, data=None, files=None):
        url = f"{self.base_url}{endpoint}"
        headers = {"Authorization": f"Bearer {self.token}", "Accept": "application/json"}
        json_payload = None

        if not files:
            headers["Content-Type"] = "application/json"
            json_payload = data
        
        try:
            response = requests.request(method, url, headers=headers, params=params, json=json_payload, files=files, timeout=30)
            response.raise_for_status()
            if response.status_code == 204:
                return {"success": True, "data": None}
            return {"success": True, "data": response.json()}
        except requests.exceptions.RequestException as e:
            return handle_api_error(e, f"{method} {endpoint}")

    def find_task_by_wip(self, wip_number, opt_fields="name,gid,parent,memberships"):
        params = {"text": wip_number, "resource.type": "task", "opt_fields": opt_fields}
        result = self._make_request('GET', f"/workspaces/{self.workspace_id}/tasks/search", params=params)
        if result["success"] and result["data"]:
            if result["data"].get("data"):
                return {"success": True, "task_data": result["data"]["data"][0]}
            else:
                return {"success": False, "message": f"No task found with WIP: '{wip_number}'."}
        return result

    def get_tasks_by_tag(self, tag_gid, opt_fields="name,gid"):
        params = {"opt_fields": opt_fields}
        return self._make_request('GET', f"/tags/{tag_gid}/tasks", params=params)

    def get_task_details(self, task_gid, opt_fields="name,gid"):
        return self._make_request('GET', f"/tasks/{task_gid}", params={"opt_fields": opt_fields})

    def get_subtasks_for_task(self, parent_task_id):
        return self._make_request('GET', f"/tasks/{parent_task_id}/subtasks", params={"opt_fields": "name,gid"})

    def add_tag_to_task(self, task_id, tag_id):
        return self._make_request('POST', f"/tasks/{task_id}/addTag", data={"data": {"tag": tag_id}})

    def remove_tag_from_task(self, task_id, tag_id):
        return self._make_request('POST', f"/tasks/{task_id}/removeTag", data={"data": {"tag": tag_id}})

    def assign_task_to_user(self, task_id, assignee_gid):
        return self._make_request('PUT', f"/tasks/{task_id}", data={"data": {"assignee": assignee_gid}})

    def add_comment_to_task(self, task_id, comment_text):
        return self._make_request('POST', f"/tasks/{task_id}/stories", data={"data": {"text": comment_text}})

    def change_task_name(self, task_id, new_name):
        return self._make_request('PUT', f"/tasks/{task_id}", data={"data": {"name": new_name}})

    def move_task_to_section(self, task_id, target_section_id):
        return self._make_request('POST', f"/sections/{target_section_id}/addTask", data={"data": {"task": task_id}})
    
    def upload_attachment(self, parent_gid, file_data):
        logging.info(f"Uploading attachment to parent GID: {parent_gid}")
        if isinstance(file_data, dict):
            try:
                files_payload = {'file': (file_data['file_name'], file_data['file_content'], file_data.get('content_type', 'application/octet-stream'))}
                return self._make_request('POST', f"/tasks/{parent_gid}/attachments", files=files_payload)
            except KeyError as e:
                return {"success": False, "message": f"Missing required file data: {e}"}
            except Exception as e:
                return {"success": False, "message": f"Error processing in-memory file: {e}"}
        else:
            return {"success": False, "message": "Invalid file data type for upload."}