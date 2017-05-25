import pickle
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

from alexnet import AlexNet

nb_classes = 43
epochs = 10
batch_size = 128

# TODO: Load traffic signs data.
with open('train.p', mode='rb') as f:
	data = pickle.load(f)

# TODO: Split data into training and validation sets.
X_train, X_val, y_train, y_val = train_test_split(data['features'], data['labels'], test_size=0.33, random_state=0)


# TODO: Define placeholders and resize operation.
features = tf.placeholder(tf.float32, (None, 32, 32, 3))
labels = tf.placeholder(tf.int64, (None))
resized = tf.image.resize_images(features, (227, 227))

# TODO: pass placeholder as first argument to `AlexNet`.
fc7 = AlexNet(resized, feature_extract=True)
# NOTE: `tf.stop_gradient` prevents the gradient from flowing backwards
# past this point, keeping the weights before and up to `fc7` frozen.
# This also makes training faster, less work to do!
fc7 = tf.stop_gradient(fc7)

# TODO: Add the final layer for traffic sign classification.
shape = (fc7.get_shape().as_list()[-1], nb_classes)
fc8W = tf.Variable(tf.truncated_normal(shape, stddev=1e-2))
fc8b = tf.Variable(tf.zeros(nb_classes))
logits = tf.matmul(fc7, fc8W) + fc8b

# TODO: Define loss, training, accuracy operations.
# HINT: Look back at your traffic signs project solution, you may
# be able to reuse some the code.
cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=logits, labels=labels)
loss_operation = tf.reduce_mean(cross_entropy)
optimizer = tf.train.AdamOptimizer()
training_operation = optimizer.minimize(loss_operation, var_list=[fc8W, fc8b])

predictions = tf.argmax(logits, 1)
accuracy_operation = tf.reduce_mean(tf.cast(tf.equal(predictions, labels), tf.float32))

def evaluate(X, y, sess):
	total_accuracy = 0
	for offset in (0, X.shape[0], batch_size):
		end = offset + batch_size
		batch_x, batch_y = X[offset:end], y[offset:end]
		accuracy = sess.run(accuracy_operation, feed_dict={features: batch_x, labels: batch_y})
		total_accuracy += (accuracy * batch_x.shape[0])
	return total_accuracy / X.shape[0]


# TODO: Train and evaluate the feature extraction model.
with tf.Session() as sess:
	sess.run(tf.global_variables_initializer())

	print('Training...')
	print()

	for i_epoch in range(epochs):
		X_train, y_train = shuffle(X_train, y_train)

		for offset in range(0, X_train.shape[0], batch_size):
			end = offset + batch_size
			batch_x, batch_y = X_train[offset:end], y_train[offset:end]
			sess.run(training_operation, feed_dict={features: batch_x, labels: batch_y})

		validation_accuracy = evaluate(X_val, y_val, sess)
		print('Epoch %i - Validation accuracy: %3.1f%%' % (i_epoch, 100*validation_accuracy))
