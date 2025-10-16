# telegram_manager.py (Revised for stability and color correction)

import asyncio
import telegram
from telegram.ext import Application, CallbackQueryHandler
import os
import cv2 # Import OpenCV for color correction

### ?? IMPORTANT: Your bot information ###
TELEGRAM_TOKEN = "8092729273:AAFTo9hY14O15zy_as1LxbJpoWCi3qtMGrQ"
ADMIN_CHAT_ID = "6696841346"
### -------------------------------- ###

REQUEST_FLAG = 'request_telegram.flag'
RESPONSE_FLAG = 'response_telegram.flag'
IMAGE_TO_SEND = "unauthorized.jpg"
CORRECTED_IMAGE_PATH = "unauthorized_corrected.jpg" # Path for the color-corrected image

async def button_callback(update, context):
    """ Handles responses from Telegram button clicks. """
    query = update.callback_query
    await query.answer()
    
    with open(RESPONSE_FLAG, "w") as f:
        f.write(query.data)
    
    await query.edit_message_text(text=f"Selection confirmed: {query.data}")
    print(f"?? Admin response received: {query.data}")

async def send_alert(context: telegram.ext.ContextTypes.DEFAULT_TYPE):
    """ Job function that sends the alert photo and buttons. """
    try:
        # ? Color Correction Step
        # Read the BGR image saved by the main script
        bgr_image = cv2.imread(IMAGE_TO_SEND)
        # Convert it to RGB, which Telegram displays correctly
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        # Save the color-corrected image temporarily
        cv2.imwrite(CORRECTED_IMAGE_PATH, rgb_image)

        caption = "Unauthorized user detected. Grant access to the safe?"
        
        keyboard = [[telegram.InlineKeyboardButton("? Approve", callback_data='approved'),
                     telegram.InlineKeyboardButton("? Reject", callback_data='rejected')]]
        reply_markup = telegram.InlineKeyboardMarkup(keyboard)
        
        # Send the color-corrected photo
        await context.bot.send_photo(chat_id=ADMIN_CHAT_ID, photo=open(CORRECTED_IMAGE_PATH, 'rb'), caption=caption, reply_markup=reply_markup)
        
        print("?? Authentication request photo sent via Telegram.")

    except Exception as e:
        print(f"[ERROR] Failed to send Telegram message: {e}")
    finally:
        # Clean up the temporary corrected image file
        if os.path.exists(CORRECTED_IMAGE_PATH):
            os.remove(CORRECTED_IMAGE_PATH)


async def main():
    """ Runs the Telegram bot and watches for the request flag. """
    if os.path.exists(RESPONSE_FLAG):
        os.remove(RESPONSE_FLAG)
    
    application = Application.builder().token(TELEGRAM_TOKEN).build()
    application.add_handler(CallbackQueryHandler(button_callback))
    
    # Run the bot in the background
    await application.initialize()
    await application.start()
    await application.updater.start_polling()
    
    print("?? Telegram bot is running and waiting for requests...")

    # Main loop to watch for the request flag
    try:
        while True:
            if os.path.exists(REQUEST_FLAG):
                # Run the send_alert job once, immediately
                application.job_queue.run_once(send_alert, 0)
                os.remove(REQUEST_FLAG) # Remove the flag *after scheduling* the job
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down bot...")
    finally:
        await application.updater.stop()
        await application.stop()
        await application.shutdown()
        print("Telegram bot shut down.")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, SystemExit):
        print("\nProgram exiting.")
