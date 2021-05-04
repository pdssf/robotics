# Para rodar no windows

Só abrir o .ttt no coppelia, iniciar a simulação (botão play) e rodar o script em python 
no cmd ou powershell normalmente.

# Para rodar no linux

Em teoria é igual, porém ainda não testei...

# OBS

Antes de iniciar a simulação no coppelia deve-se colocar a linha simRemoteApi.start(19999), dentro de qualquer 
script da cena, de preferência o script principal, como descrito na imagem cop_change, que se encontra no diretório 
api_python (no .ttt que se encrontra dentro do diretório windows isso já foi feito).
