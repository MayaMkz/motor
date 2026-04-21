¡Qué onda! Claro que sí. Me di a la tarea de extraer los códigos y arreglar un par de errorcitos de sintaxis que se generan al copiar directo del PDF (como símbolos faltantes por el formato del texto) para que te funcionen directo a la primera.

[cite_start]Ojo: El documento en realidad tiene **3 archivos**[cite: 126, 203, 205]. [cite_start]El Jupyter Notebook necesita que exista la clase `NeuralNetwork` en un archivo separado para poder importarla[cite: 211]. 

Aquí tienes los tres, listos para copiar y pegar:

### 1. Generador de Datos (`dataGeneration.py`)
[cite_start]Este es el que simula el péndulo y te crea el archivo `dataPendulo.csv`[cite: 125, 126].

```python
import numpy as np
import pandas as pd

tf = 60
h = 0.001
b = 0.05
l = 1
m = 1
g = 9.81

integral = 0
prev_error = 0

time = np.linspace(0, tf, int(tf/h))
x1 = np.zeros_like(time)
x2 = np.zeros_like(time)
u = np.zeros_like(time)
ref = np.zeros_like(time)
k = 0

def generateReferencePRBS():
    minT = 0.1
    maxT = 0.5
    T = np.random.uniform(minT, maxT, size=int(tf/minT))
    ref = np.zeros_like(time)
    current_time = 0
    for t in T:
        ref[current_time:current_time+int(t/h)] = 2*np.random.rand() - 1
        current_time += int(t/h)
        if current_time >= len(time):
            break
    return ref

def PID():
    global integral, prev_error
    Kp = 100
    Ki = 0
    Kd = 1
    
    error = ref[k] - x1[k]
    integral = integral + error*h
    derivative = (error - prev_error)/h
    prev_error = error
    return Kp*error + Ki*integral + Kd*derivative

if __name__ == "__main__":
    x1[0] = 0.1
    x2[0] = 0
    ref = generateReferencePRBS()
    
    for k in range(0, len(time)-1):
        u[k] = PID()
        x1[k+1] = x1[k] + h*x2[k]
        x2[k+1] = x2[k] + h*(m*g*l*np.sin(x1[k]) - b*x2[k] + u[k])/m
        
    pd.DataFrame({'time': time, 'x1': x1, 'x2': x2, 
                  'u': u, 'ref': ref}).to_csv('dataPendulo.csv', index=False)
```

### 2. Clase de la Red Neuronal (`NeuralNetwork.py`)
Tienes que guardar este código en el mismo directorio con el nombre `NeuralNetwork.py`. [cite_start]El Jupyter Notebook lo va a mandar llamar[cite: 203, 211].

```python
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

class NeuralNetwork(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(NeuralNetwork, self).__init__()
        self.lin1 = nn.Linear(input_size, hidden_size)
        self.lin2 = nn.Linear(hidden_size, hidden_size)
        self.lin3 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        x = torch.sigmoid(self.lin1(x))
        x = torch.tanh(self.lin2(x))
        x = self.lin3(x)
        return x

    def fit(self, x_train, y_train, epochs=1000, batch_size=100, learning_rate=0.001):
        dataset = TensorDataset(x_train, y_train)
        loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(self.parameters(), lr=learning_rate)
        
        for epoch in range(epochs):
            for x_batch, y_batch in loader:
                optimizer.zero_grad()
                outputs = self.forward(x_batch)
                loss = criterion(outputs, y_batch)
                loss.backward()
                optimizer.step()
            
            # Quita el comentario de la siguiente línea si quieres ver cómo baja el error
            # print(f'Epoch [{epoch+1}/{epochs}], Loss: {loss.item():.4f}')
```

### 3. Notebook de Entrenamiento (`dinamicApproximation.ipynb`)
[cite_start]Aquí te dejo el código dividido por las celdas que indica tu documento[cite: 205]. Cópialo bloque por bloque en tu Jupyter. 
[cite_start]*(Nota: Ya le integré la corrección que sugiere el documento en el "Cell 2" para que PyTorch no te arroje alertas al cargar los datos [cite: 266, 267]).*

[cite_start]**Celda 1:** [cite: 206]
```python
import torch
import torch.nn as nn
from NeuralNetwork import NeuralNetwork
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

model = NeuralNetwork(3, 10, 1)
print(model)
```

[cite_start]**Celda 2:** [cite: 221]
```python
dataSet = pd.read_csv('dataPendulo.csv')

tD = pd.DataFrame()
tD['uk'] = dataSet['u']
tD['x2k'] = dataSet['x2']
tD['x1k'] = dataSet['x1']
tD['x1kp1'] = dataSet['x1'].shift(periods=-1)
tD['t'] = dataSet['time']

tD = tD.dropna()

uk = tD['uk'].values
x2k = tD['x2k'].values
x1k = tD['x1k'].values
x1kp1 = tD['x1kp1'].values
t = tD['t'].values

# Cargando tensores de forma optimizada
x = torch.tensor(np.column_stack([uk, x2k, x1k]), dtype=torch.float32)
y = torch.tensor(x1kp1.reshape(-1, 1), dtype=torch.float32)
```

[cite_start]**Celda 3:** [cite: 272]
```python
n_epochs = 100
batch_size = 1000
model.fit(x, y, n_epochs, batch_size, 0.01)
```

[cite_start]**Celda 4:** [cite: 280]
```python
predictions = model.forward(x)

fig, ax = plt.subplots()
ax.plot(t, x1kp1, label='Actual')
ax.plot(t, predictions.detach().numpy(), label='Predicted')

ax.set_xlabel('Time (s)')
ax.set_ylabel('$x_1$ (rad)')
ax.set_title('Actual vs Predicted $x_1$')
ax.legend()
ax.grid()
ax.set_xlim(25, 30)

plt.show()
```

¡Guarda todo, corre primero el generador en tu terminal y luego pásate a VS Code a correr tu notebook!
