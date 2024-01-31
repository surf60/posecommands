import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from keras.layers import Dropout
from datetime import datetime
import joblib

data = pd.read_csv("bigdata.csv")
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
threshold = 0.65
df = pd.DataFrame(data, columns=['state','left_shoulder.x','left_shoulder.y','left_shoulder.z','left_shoulder.visibility','right_shoulder.x','right_shoulder.y','right_shoulder.z','right_shoulder.visibility','left_elbow.x','left_elbow.y','left_elbow.z','left_elbow.visibility','right_elbow.x','right_elbow.y','right_elbow.z','right_elbow.visibility','left_wrist.x','left_wrist.y','left_wrist.z','left_wrist.visibility','right_wrist.x','right_wrist.y','right_wrist.z','right_wrist.visibility'])

#normaliseation
coordinates_columns = ['left_shoulder.x','left_shoulder.y','left_shoulder.z','left_shoulder.visibility','right_shoulder.x','right_shoulder.y','right_shoulder.z','right_shoulder.visibility','left_elbow.x','left_elbow.y','left_elbow.z','left_elbow.visibility','right_elbow.x','right_elbow.y','right_elbow.z','right_elbow.visibility','left_wrist.x','left_wrist.y','left_wrist.z','left_wrist.visibility','right_wrist.x','right_wrist.y','right_wrist.z','right_wrist.visibility']
coordinates = df[coordinates_columns]
normalized_coordinates = coordinates.copy()
normalized_coordinates['left_shoulder.x']=normalized_coordinates['left_shoulder.x']/(1/normalized_coordinates['left_shoulder.z'])
normalized_coordinates['left_shoulder.y']=normalized_coordinates['left_shoulder.y']/(1/normalized_coordinates['left_shoulder.z'])
normalized_coordinates['right_shoulder.x']=normalized_coordinates['right_shoulder.x']/(1/normalized_coordinates['right_shoulder.z'])
normalized_coordinates['right_shoulder.y']=normalized_coordinates['right_shoulder.y']/(1/normalized_coordinates['right_shoulder.z'])
normalized_coordinates['left_elbow.x']=normalized_coordinates['left_elbow.x']/(1/normalized_coordinates['left_elbow.z'])
normalized_coordinates['left_elbow.y']=normalized_coordinates['left_elbow.y']/(1/normalized_coordinates['left_elbow.z'])
normalized_coordinates['right_elbow.x']=normalized_coordinates['right_elbow.x']/(1/normalized_coordinates['right_elbow.z'])
normalized_coordinates['right_elbow.y']=normalized_coordinates['right_elbow.y']/(1/normalized_coordinates['right_elbow.z'])
normalized_coordinates['left_wrist.x']=normalized_coordinates['left_wrist.x']/(1/normalized_coordinates['left_wrist.z'])
normalized_coordinates['left_wrist.y']=normalized_coordinates['left_wrist.y']/(1/normalized_coordinates['left_wrist.z'])
normalized_coordinates['right_wrist.x']=normalized_coordinates['right_wrist.x']/(1/normalized_coordinates['right_wrist.z'])
normalized_coordinates['right_wrist.y']=normalized_coordinates['right_wrist.y']/(1/normalized_coordinates['right_wrist.z'])

#print(normalized_coordinates.head(2))
data.update(normalized_coordinates)
df = df.drop(['left_shoulder.z', 'right_shoulder.z', 'left_elbow.z', 'right_elbow.z', 'left_wrist.z', 'right_wrist.z'], axis=1)
df = df.drop(['left_shoulder.visibility','right_shoulder.visibility','left_elbow.visibility', 'right_elbow.visibility', 'left_wrist.visibility', 'right_wrist.visibility'], axis=1)

#train test data
X = df.drop(columns=['state'])  # Features
y = df['state']  # Target

label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(df['state'])
y_categorical = to_categorical(y_encoded)

#Add the one-hot encoded labels to the DataFrame
df_encoded = pd.concat([df.drop(columns=['state']), pd.DataFrame(y_categorical, columns=label_encoder.classes_)], axis=1)

X_multiclass = df_encoded.drop(columns=label_encoder.classes_)  # Features
y_multiclass = df_encoded[label_encoder.classes_]  # Target

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X_multiclass, y_multiclass, test_size=0.2, random_state=42)

num_classes = len(label_encoder.classes_)

model = Sequential()
model.add(Dense(256, activation='relu', input_shape=(X_train.shape[1],)))
model.add(Dropout(0.5))
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(64, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(num_classes, activation='softmax'))

model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_test, y_test))

accuracy = model.evaluate(X_test, y_test)[1]
print(f"Accuracy: {accuracy*100:.2f}%")

joblib.dump(model, f'{current_time}_model_{accuracy*100}.pkl')