#in powershell of windows:-
# Create certs inside mavlink-mtls folder folder if missing
New-Item -ItemType Directory -Force -Path "certs" | Out-Null
Set-Location certs

Write-Host "Generating CA..."
openssl req -x509 -newkey rsa:4096 -keyout ca.key -out ca.crt -sha256 -days 365 -nodes -subj "/CN=MyCA"

Write-Host "Generating Server Key + CSR..."
openssl req -newkey rsa:4096 -keyout server.key -out server.csr -nodes -subj "/CN=GCS-Server"

Write-Host "Signing Server Certificate..."
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days 365 -sha256

Write-Host "Generating Client Key + CSR..."
openssl req -newkey rsa:4096 -keyout client.key -out client.csr -nodes -subj "/CN=Drone-Client"

Write-Host "Signing Client Certificate..."
openssl x509 -req -in client.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out client.crt -days 365 -sha256

Write-Host "Creating PEM bundles..."
Copy-Item server.key -Destination server-combined.pem
Get-Content server.crt | Add-Content server-combined.pem

Copy-Item client.key -Destination client-combined.pem
Get-Content client.crt | Add-Content client-combined.pem

Write-Host "`n✔ Certificates generated successfully in /certs folder!"



#for executing it in windows powershell type below command line by line:-
''' cd C:\Users\HP\mavlink-mtls
Set-ExecutionPolicy Bypass -Scope Process -Force
powershell -ExecutionPolicy Bypass -File .\create_certs.ps1'''
