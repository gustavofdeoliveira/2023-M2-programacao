# Semana 05

### Links:

Video: [https://drive.google.com/file/d/1GWr_yMHBby3RLEVzIpaB5l9KLhjZo4OH/view?usp=share_link](https://drive.google.com/file/d/1GWr_yMHBby3RLEVzIpaB5l9KLhjZo4OH/view?usp=share_link)

Notebook: [https://colab.research.google.com/drive/16dx94nR-vl7ccJhxRIZRI7oDtVbxk7gd?usp=sharing](https://colab.research.google.com/drive/16dx94nR-vl7ccJhxRIZRI7oDtVbxk7gd?usp=sharing)

### Descrição:

Para a elaboração da atividade desta semana, foi necessário aplicar dois tipos de conhecimento vistos: a utilização do OpenCV e o YOLO.

Para começar, criei um arquivo no Colab para ter um ambiente de trabalho melhor. A primeira etapa foi criar uma célula para verificar o acesso à GPU e usá-la. Em seguida, fiz a instalação e importação do YOLO, pois ele será responsável por treinar e criar nosso modelo de predição. Para confirmar se a biblioteca estava funcionando, criei uma parte de teste para detecção de cachorros e verifiquei as métricas do modelo.

Após a confirmação de funcionamento, comecei a trabalhar na importação do dataset do Roboflow de rachaduras (crack). Como tinha visto que o teste anterior estava bom, utilizei-o como base para o de rachaduras. Realizei o treinamento e, em seguida, dei uma olhada nas métricas para verificar se havia necessidade de melhorar (no caso, não havia) e vi os resultados obtidos.

Por fim, criei um arquivo main.py, onde utilizei o OpenCV para acessar a câmera e utilizar o modelo na detecção de rachaduras durante o vídeo.
