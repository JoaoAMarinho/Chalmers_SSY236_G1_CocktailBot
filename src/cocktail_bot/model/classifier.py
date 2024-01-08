from sklearn.preprocessing import OneHotEncoder
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier, plot_tree
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report, ConfusionMatrixDisplay

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from joblib import dump

# Must be executed from the source folder
PATH = "./src/cocktail_bot/model/"

# Read training data
df = pd.read_csv(PATH + "datasets/data.csv")

# Encode categorical data
encoder = OneHotEncoder(handle_unknown="ignore")
encoded_data = encoder.fit_transform(df[["shape"]]).toarray()
encoded_names = encoder.get_feature_names_out(["shape"])

encoded_df = pd.DataFrame(encoded_data, columns=encoded_names)

# Concatenate encoded data with original data
df = pd.concat([df, encoded_df], axis=1)

# Extract features and target
x = df.drop(["label", "shape"], axis=1)
y = df["label"]

# Split data into training and test sets
X_train, X_test, y_train, y_test = train_test_split(
    x, y, test_size=0.2, random_state=42
)

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
matrix = confusion_matrix(y_test, predictions, labels=classifer.classes_)
fig, ax = plt.subplots(figsize=(10,10))
disp = ConfusionMatrixDisplay(confusion_matrix=matrix,display_labels=classifer.classes_)
disp.plot(
    cmap="Blues",
    xticks_rotation='vertical',
    ax=ax)
plt.title("Confusion Matrix")
plt.xlabel("Predicted")
plt.ylabel("Actual")
plt.savefig(PATH + "results/confusion_matrix.png")
plt.close

# Report
classification_rep = classification_report(y_test, predictions, output_dict=True)
rep_df = pd.DataFrame(classification_rep).transpose()
rep_df.to_csv(PATH + "results/classification_report.csv")

# Decision tree structure
plt.figure(figsize=(20, 10))
plot_tree(
    classifer, filled=True, feature_names=x.columns, class_names=df["label"].unique()
)
plt.savefig(PATH + "results/decision_tree.png")
plt.close


classifer = DecisionTreeClassifier()
classifer.fit(x, y)
dump(classifer, PATH + "classifier.joblib")
# load(path + 'classifier.joblib')
