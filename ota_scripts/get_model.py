from googleapiclient.discovery import build
from googleapiclient.http import MediaIoBaseDownload
from google.oauth2.credentials import Credentials
import io
import os

# Load your credentials from the token file
creds = Credentials.from_authorized_user_file('token.json', scopes=['https://www.googleapis.com/auth/drive.file'])

service = build('drive', 'v3', credentials=creds)

def find_and_download_file(filename):
    # Search for the file by name
    results = service.files().list(q=f"name='{filename}'", spaces='drive', fields="files(id, name)").execute()
    items = results.get('files', [])

    if not items:
        print('No files found.')
        return
    else:
        for item in items:
            print(f"Downloading {item['name']}...")
            request = service.files().get_media(fileId=item['id'])
            fh = io.BytesIO()
            downloader = MediaIoBaseDownload(fh, request)
            done = False
            while done is False:
                status, done = downloader.next_chunk()
                print(f"Download {int(status.progress() * 100)}%.")
            fh.seek(0)
            
            with open(filename, 'wb') as f:
                f.write(fh.read())
            print(f"{filename} has been downloaded.")

# Replace 'model.cc' with the actual name of your file
find_and_download_file('model.cc')