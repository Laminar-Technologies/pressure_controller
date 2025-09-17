# Asana_Imports/asana_error_handler.py
import requests
import logging

def handle_api_error(e, operation_name):
    error_message = f"An unexpected error occurred during {operation_name}."
    if isinstance(e, requests.exceptions.HTTPError):
        status_code = e.response.status_code
        response_text = e.response.text
        logging.error(f"HTTP error {status_code} during {operation_name}: {e}. Response: {response_text}")
        if status_code == 400:
            error_message = f"Error 400: Bad Request. Check input data or request format. Details: {response_text}"
        elif status_code == 401:
            error_message = "Error 401: Unauthorized. Check your Asana PAT."
        elif status_code == 403:
            error_message = f"Error 403: Forbidden. Insufficient permissions for {operation_name}. Details: {response_text}"
        elif status_code == 404:
            error_message = f"Error 404: Not Found. Verify IDs or endpoint. Details: {response_text}"
        elif status_code == 429:
            retry_after = e.response.headers.get('Retry-After', 'N/A')
            error_message = f"Warning 429: Too Many Requests. Rate limit exceeded. Try again in {retry_after} seconds."
        else:
            error_message = f"An unexpected HTTP error {status_code} occurred: {e.response.reason}. Details: {response_text}"
    elif isinstance(e, requests.exceptions.ConnectionError):
        logging.error(f"Network connection error during {operation_name}: {e}")
        error_message = "A network connection error occurred. Please check your internet connection."
    elif isinstance(e, requests.exceptions.Timeout):
        logging.error(f"Request timed out during {operation_name}: {e}") 
        error_message = "The request to Asana API timed out. This might indicate a slow API response or network issue."
    elif isinstance(e, ValueError):
        logging.error(f"Failed to parse JSON response for {operation_name}: {e}. Raw response: {e.response.text if hasattr(e, 'response') else 'N/A'}")
        error_message = f"Failed to parse Asana API response for {operation_name}."
    elif isinstance(e, requests.exceptions.RequestException):
        logging.error(f"An unexpected general requests error occurred during {operation_name}: {e}")
        error_message = f"An unexpected error occurred during the API request: {e}"
    else:
        logging.error(f"An unknown error occurred during {operation_name}: {e}", exc_info=True)
        error_message = f"An unknown error occurred during {operation_name}."
    return {"success": False, "message": error_message}