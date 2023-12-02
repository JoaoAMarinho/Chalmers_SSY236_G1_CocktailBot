from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier, plot_tree
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

#Must be executed from the rep folder
data = pd.read_csv('./src/cocktail_bot/dataset/data.csv')
print(data.head())
x = data.drop(['label','shape'], axis=1)  # Features

y = data['label'] # Target

X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)

# Create a Decision Tree Classifier
classifer = DecisionTreeClassifier()

# Train the classifier on the training data
classifer.fit(X_train, y_train)

# Make predictions on the test data
predictions = classifer.predict(X_test)

# Calculate accuracy
accuracy = accuracy_score(y_test, predictions)
print(f"Accuracy: {accuracy * 100:.2f}%")

# Create a confusion matrix
matrix = confusion_matrix(y_test, predictions)
plt.figure(figsize=(10, 10))
sns.heatmap(matrix, annot=True, fmt='d', cmap='Blues', xticklabels=data['label'].unique(), yticklabels=data['label'].unique())
plt.title('Confusion Matrix')
plt.xlabel('Predicted')
plt.ylabel('Actual')
plt.savefig("cm.png")
plt.close

#Report
classification_rep = classification_report(y_test, predictions)
print(f'Classification Report:\n{classification_rep}')

#Decision tree structure
plt.figure(figsize=(20, 10))
plot_tree(classifer, filled=True, feature_names=x.columns, class_names=data['label'].unique())
plt.savefig("dt.png")
plt.close

