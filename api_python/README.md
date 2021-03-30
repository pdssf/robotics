Pra rodar no windows, só abrir o .ttt no coppelia, iniciar a simulação (botão play) e rodar o script em python 
no cmd ou powershell normalmente (por enquanto não fiz o scrip do robô diferencial, mas é possível testar usando 
o simpleTest.py nos diretórios windows ou linux).
No linux, em teoria é igual, porém ainda não testei...

Ah, e antes de iniciar a simulação no coppelia deve-se colocar a linha simRemoteApi.start(19999), dentro de qualquer 
script da cena, de preferência o script principal, como descrito na imagem cop_change, que se encontra no diretório 
api_python(no .ttt que se encrontra dentro do diretório api_python isso já foi feito, portanto é  só abri-lo e apertar 
play).
