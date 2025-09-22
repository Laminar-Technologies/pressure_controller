# Asana_Imports/asana_logic.py
import os
import shutil
from tkinter import messagebox
from .asana_api_client import AsanaClient

def find_and_validate_tasks(client, wip_number):
    """Finds and validates the parent and subtask for a given WIP number."""
    opt_fields = "name,gid,parent,projects.gid,tags.gid"
    initial_task_result = client.find_task_by_wip(wip_number, opt_fields=opt_fields)
    if not initial_task_result["success"]:
        return initial_task_result
    task_data = initial_task_result["task_data"]
    parent_gid = None
    subtask_gid = None
    if task_data.get('parent'):
        parent_gid = task_data['parent']['gid']
        subtask_gid = task_data['gid']
    else:
        parent_gid = task_data['gid']
        subtasks_result = client.get_subtasks_for_task(parent_gid)
        if not subtasks_result["success"]:
            return subtasks_result
        subtasks = subtasks_result.get("data", {}).get("data", [])
        matching_subtask = next((st for st in subtasks if wip_number.lower() in st.get('name', '').lower()), None)
        if not matching_subtask:
            return {"success": False, "message": f"No subtask for '{wip_number}' found under the main task."}
        subtask_gid = matching_subtask['gid']
    return {"success": True, "parent_gid": parent_gid, "subtask_gid": subtask_gid}

def upload_cert_to_asana(wip_number, cert_path, client, gids):
    """
    Uploads a calibration certificate to Asana, updates task assignments, and moves the project.
    """
    try:
        # Find the parent and subtask GIDs
        task_validation = find_and_validate_tasks(client, wip_number)
        if not task_validation["success"]:
            messagebox.showerror("Asana Error", f"Could not find a valid task for WIP {wip_number}.\n\n{task_validation['message']}")
            return

        parent_gid = task_validation["parent_gid"]
        subtask_gid = task_validation["subtask_gid"]

        # 1. Upload the certificate to the subtask
        with open(cert_path, 'rb') as file_content:
            file_data = {
                "file_name": os.path.basename(cert_path),
                "file_content": file_content.read(),
                "content_type": "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"
            }
            upload_result = client.upload_attachment(subtask_gid, file_data)
            if not upload_result["success"]:
                messagebox.showerror("Asana Error", f"Failed to upload certificate for {wip_number}.\n\n{upload_result['message']}")
                return

        # 2. Assign the subtask to Michelle Hughes
        assign_subtask_result = client.assign_task_to_user(subtask_gid, gids["SHARED_SUBTASK_ASSIGNEE"])
        if not assign_subtask_result["success"]:
            messagebox.showwarning("Asana Warning", f"Could not assign subtask for {wip_number} to Michelle Hughes.\n\n{assign_subtask_result['message']}")

        # 3. Assign the main task to Mandy McIntosh
        assign_parent_result = client.assign_task_to_user(parent_gid, gids["ACCOUNT_MANAGER_ASSIGNEE"])
        if not assign_parent_result["success"]:
            messagebox.showwarning("Asana Warning", f"Could not assign main task for {wip_number} to Mandy McIntosh.\n\n{assign_parent_result['message']}")

        # 4. Move the main task to the 'Ready for Buyer' section
        move_result = client.move_task_to_section(parent_gid, gids["READY_FOR_BUYER_SECTION"])
        if not move_result["success"]:
            messagebox.showwarning("Asana Warning", f"Could not move main task for {wip_number} to 'Ready for Buyer'.\n\n{move_result['message']}")

        # 5. Copy the certificate to OneDrive
        onedrive_path = r"C:\Users\trnc\OneDrive - Laminar Technologies\Laminar Technologies - Documents\Cal Certs (1)\Baratron (One Drive)"
        if os.path.exists(onedrive_path):
            shutil.copy(cert_path, onedrive_path)
        else:
            messagebox.showwarning("OneDrive Warning", f"Could not find the OneDrive directory:\n{onedrive_path}\n\nThe certificate for {wip_number} was not copied.")

        messagebox.showinfo("Asana Upload Complete", f"Successfully uploaded certificate and updated Asana for WIP {wip_number}.")

    except Exception as e:
        messagebox.showerror("Asana Error", f"An unexpected error occurred while processing {wip_number}.\n\n{e}")